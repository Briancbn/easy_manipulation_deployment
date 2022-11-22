// Copyright 2021 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "emd/dynamic_safety/dynamic_safety.hpp"
#include "emd/dynamic_safety/dynamic_safety_options.hpp"
#include "emd/dynamic_safety/dynamic_safety_impl.hpp"

namespace dynamic_safety
{

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("dynamic_safety");
static const double LOG_RATE = 1;  // This is duration

void DynamicSafety::Impl::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  joint_names.clear();
  for (auto & joint : rt->joint_names) {
    joint_names.insert(joint);
  }
  collision_checker_.add_trajectory(rt);
  full_duration_ = rclcpp::Duration(rt->points.back().time_from_start).seconds();
  if (option_.visualize) {
    visualizer_.add_trajectory(rt);
  }
  if (option_.allow_replan) {
    replanner_.add_trajectory(rt);
  }
  activated_ = true;
}


void DynamicSafety::Impl::update_time(double current_time)
{
  current_time_cache_.writeFromNonRT(current_time);
}

void DynamicSafety::Impl::update_state(const sensor_msgs::msg::JointState::SharedPtr & state)
{
  env_state_cache_.writeFromNonRT(*state);
}

void DynamicSafety::Impl::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  current_state_cache_.writeFromNonRT(CurrentState(joint_names, current_state));
}

void DynamicSafety::Impl::update_scene(const moveit_msgs::msg::PlanningScene::SharedPtr & scene_msg)
{
  moveit_scene_cache_.writeFromNonRT(*scene_msg);
}


double DynamicSafety::Impl::get_scale()
{
  return *scale_cache_.readFromRT();
}

void DynamicSafety::Impl::start()
{
  // next_point_publisher_.start();
  // RCLCPP_INFO(LOGGER, "Next point publisher started!");
  // collision_time_point_ = -1;

  if (activated_) {
    if (!started) {
      if (option_.benchmark) {
        pf_ = new emd::TimeProfiler<>(5000);
      }

      started = true;

      if (option_.visualize) {
        visualizer_.start();
      }
      sig_ = std::promise<void>();
      future_ = sig_.get_future();
      RCLCPP_INFO(LOGGER, "All started");
    } else {
      RCLCPP_WARN(LOGGER, "Already started");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Not configured!! Please call the configure() first");
  }
}

void DynamicSafety::Impl::wait()
{
  RCLCPP_INFO(LOGGER, "Waiting...");
  future_.wait();
  RCLCPP_INFO(LOGGER, "Successfully exit.");
}

void DynamicSafety::Impl::stop()
{
  // visualizer_.reset();
  // RCLCPP_INFO(
  //   LOGGER, "Next Point Publisher ended with %s status",
  //   (next_point_publisher_.get_status() == NextPointPublisher::SUCCEEDED) ?
  //   "SUCCEEDED" : "FAILED");
  // next_point_publisher_.reset();
  if (option_.visualize) {
    visualizer_.stop();
  }
  started = false;
  activated_ = false;
  // node_.reset();

  // Print out result
  if (option_.benchmark) {
    if (pf_) {
      std::ostringstream oss;
      pf_->print(oss);
      RCLCPP_INFO_STREAM(
        LOGGER,
        "Time stats:\n" << oss.str());
      delete pf_;
    }
  }
  sig_.set_value();
}

void DynamicSafety::Impl::_deadline_cb(rclcpp::QOSDeadlineRequestedInfo &)
{
}

void DynamicSafety::Impl::_main_loop()
{
  // Update joint state
  if (!option_.environment_joint_states_topic.empty()) {
    collision_checker_.update(*env_state_cache_.readFromRT());
    if (option_.allow_replan) {
      replanner_.update(*env_state_cache_.readFromRT());
    }
  }

  // Scene with MoveIt Scene
  if (!option_.moveit_scene_topic.empty()) {
    collision_checker_.update(*moveit_scene_cache_.readFromRT());
    if (option_.allow_replan) {
      replanner_.update(*moveit_scene_cache_.readFromRT());
    }
  }

  // get scaled time point
  double current_time_point = *current_time_cache_.readFromRT();
  // RCLCPP_INFO(node_->get_logger(), "Current time: %f", current_time_point);
  collision_time_point_ = -1;

  // Check collision once
  collision_checker_.run_once(
    current_time_point,
    option_.safety_zone_options.look_ahead_time,
    collision_time_point_
  );

  double scale = *scale_cache_.readFromRT();

  // Dynamically adjust slow down time
  if (option_.safety_zone_options.slow_down_time <= 0 && option_.dynamic_parameterization) {
    SafetyZone::Option dynamic_option = option_.safety_zone_options;
    if (scale != 0.0001) {
      dynamic_option.slow_down_time =
        _cal_scale_time(*current_state_cache_.readFromRT(), scale, 0.0001);
    } else {
      dynamic_option.slow_down_time = 0;
    }

    // Dynamically set scale step
    if (dynamic_option.slow_down_time > 0) {
      safety_zone_.set(dynamic_option);

      // Update visualizer as well
      if (option_.visualize) {
        visualizer_.update(dynamic_option);
      }
    } else if (scale != 0.0001) {
      RCLCPP_ERROR(
        LOGGER,
        "There is no velocity state feedback from the robot. "
        "Please check /joint_states for velocity values!!"
        "Hard set the slow down time to 0.5s instead");
      option_.safety_zone_options.slow_down_time = 0.5;
      safety_zone_.set(option_.safety_zone_options);
      // Update visualizer as well
      if (option_.visualize) {
        visualizer_.update(option_.safety_zone_options);
      }
    }
  } else {
    // preconfifured slow down time reduce it based on current scale
    SafetyZone::Option dynamic_option = option_.safety_zone_options;
    dynamic_option.slow_down_time = option_.safety_zone_options.slow_down_time * scale;
    safety_zone_.set(dynamic_option);
    // Update visualizer as well
    if (option_.visualize) {
      visualizer_.update(dynamic_option);
    }
  }

  // Collision Happens in the future
  if (collision_time_point_ >= current_time_point) {
    double scale_step = 0;
    if (option_.safety_zone_options.slow_down_time <= 0 && option_.dynamic_parameterization) {
      // Dynamically adjust slow down time
      double slow_down_time =
        _cal_scale_time(*current_state_cache_.readFromRT(), scale, 0.0001);

      scale_step = (scale - 0.0001) * (1.0 / option_.rate) / slow_down_time;
      scale_step = 2 * option_.rate;
    } else {
      // Static Scale step
      scale_step = 1 * (1.0 / option_.rate) / option_.safety_zone_options.slow_down_time;
    }
    uint8_t zone = safety_zone_.get_zone(collision_time_point_ - current_time_point);

    if (!option_.allow_replan) {
      // No replanning
      if (zone <= SafetyZone::EMERGENCY) {
        // Emergency stop
        RCLCPP_ERROR_ONCE(
          LOGGER,
          "Emergency stop!!");
        scale = 0.0001;
      } else {
        // Slow down
        RCLCPP_WARN_ONCE(
          LOGGER,
          "Slowing down!!");
        scale -= scale_step;
        scale = std::max<double>(scale, 0.0001);
      }
    } else {
      // Replanning
      double current_time = *current_time_cache_.readFromRT();
      if (zone <= SafetyZone::EMERGENCY) {
        // Emergency stop
        RCLCPP_ERROR_ONCE(
          LOGGER,
          "Emergency stop!!");
        scale = 0.0001;
      } else if (zone == SafetyZone::SLOWDOWN) {
        // TODO(anyone): Better Heuristic
        scale -= scale_step;
        double start_state_time =
          (current_time + safety_zone_.get_zone_limit(SafetyZone::EMERGENCY) +
          collision_time_point_) / 2;
        _handle_replanner(start_state_time);
      } else if (zone == SafetyZone::REPLAN) {
        double start_state_time =
          (current_time + safety_zone_.get_zone_limit(SafetyZone::SLOWDOWN) +
          collision_time_point_) / 2;
        _handle_replanner(start_state_time);
      } else if (zone == SafetyZone::SAFE) {
        auto status = replanner_.get_status();
        if (status == ReplannerStatus::IDLE) {
          // Replanner Starte
          // Nothing to do here
        } else if (status == ReplannerStatus::ONGOING) {
          replanner_.terminate_async();
        } else if (status == ReplannerStatus::TIMEOUT) {
          replanner_.terminate_async();
        } else if (status == ReplannerStatus::SUCCEED) {
          // stop it regularly
          replanner_.get_result();
        }
      }
    }
  } else {
    // No Collision
    if (scale < 1.0) {
      // Gradually rescale back to 1
      double scale_time;
      if (option_.safety_zone_options.slow_down_time <= 0 && option_.dynamic_parameterization) {
        scale_time = _cal_scale_time(*current_state_cache_.readFromRT(), scale, 1.0);
        scale += (1.0 - scale) * (1.0 / option_.rate) / scale_time;
      } else {
        scale_time = option_.safety_zone_options.slow_down_time;
        scale += 1.0 * (1.0 / option_.rate) / scale_time;
      }
      scale = std::min<double>(scale, 1);
      RCLCPP_WARN_ONCE(LOGGER, "Speeding up");
    }
  }
  scale_cache_.writeFromNonRT(scale);

  if (option_.visualize) {
    visualizer_.update(current_time_point, collision_time_point_);
  }
}

double DynamicSafety::Impl::_cal_scale_time(
  const CurrentState & current_state,
  double current_scale,
  double target_scale)
{
  double scale_time = -1.0;
  if (!current_state.state.velocities.empty()) {
    for (size_t i = 0; i < current_state.state.velocities.size(); i++) {
      // Skip joints that is not controlled by this controller
      if (joint_names.find(current_state.joint_names[i]) == joint_names.end()) {
        continue;
      }
      // Skip joints with no limits
      if (option_.joint_limits[current_state.joint_names[i]].first == 0) {
        continue;
      }
      // Skip joints with no limits
      if (option_.joint_limits[current_state.joint_names[i]].second == 0) {
        continue;
      }
      // Slow down
      if (current_scale >= target_scale) {
        double temp_scale_time =
          ::fabs(current_state.state.velocities[i] * (current_scale - target_scale)) /
          current_scale / option_.joint_limits[current_state.joint_names[i]].second;
        if (temp_scale_time > scale_time) {
          scale_time = temp_scale_time;
        }
      } else {
        // Speed up
        double temp_scale_time =
          (option_.joint_limits[current_state.joint_names[i]].first -
          ::fabs(current_state.state.velocities[i])) /
          option_.joint_limits[current_state.joint_names[i]].second;
        if (temp_scale_time > scale_time) {
          scale_time = temp_scale_time;
        }
      }
    }
  }
  return scale_time;
}

void DynamicSafety::Impl::_handle_replanner(double start_state_time)
{
  // Replanner not started
  auto status = replanner_.get_status();
  if (status == ReplannerStatus::IDLE) {
    // Replanner Started
    RCLCPP_WARN_ONCE(
      LOGGER,
      "Starting replanner [%s] with starting time point",
      option_.replanner_options.planner.c_str());
    double end_state_time = _back_track_last_collision();
    replanner_.run_async(start_state_time, end_state_time);

  } else if (status == ReplannerStatus::ONGOING) {
    // Just let it run baby.
    // TODO(anyone): Better handling?
  } else if (status == ReplannerStatus::TIMEOUT) {
    // TODO(anyone): Better termination handling?
    // There is probably nothing we can do here.
    // async_terminate() function trigger a detached termination thread
    // to quietly shut down the process.
    // Once that is done status would become idle;
    replanner_.terminate_async();
  } else if (status == ReplannerStatus::SUCCEED) {
    // Let's see if you really finished, or just failed and gaveup
    auto result = replanner_.get_result();
    if (result->points.empty()) {
      // Gosh you failure, let's restart
      double end_state_time = _back_track_last_collision();
      replanner_.run_async(start_state_time, end_state_time);
    } else {
      // Good job replanner, let's add a starting point and
      // do time_parameterization.
      auto joint_names = current_state_cache_.readFromRT()->joint_names;
      auto current_state = current_state_cache_.readFromRT()->state;
      double current_time = *current_time_cache_.readFromRT();
      // Get time parameterized result
      auto new_traj = replanner_.flatten_result(current_time, joint_names, current_state);
      if (!new_traj->points.empty()) {
        NewTrajectoryCB(new_traj);
      }
    }
  }
}
double DynamicSafety::Impl::_back_track_last_collision()
{
  // Use collision checker to backtrack collision
  // This is not nearly as efficient right now to be improved.
  // TODO(anyone): Enable this in collision checker
  double time_from_start = full_duration_;
  double step = option_.collision_checker_options.step;
  double collision_time = -1;
  while (time_from_start >= 0) {
    time_from_start -= step;
    collision_checker_.run_once(time_from_start, 0.0, collision_time);
    if (collision_time > 0) {
      // Tesseract doesn't see to work well with short segment
      if (option_.replanner_options.framework == "tesseract") {
        return std::min<double>(time_from_start + 0.8, full_duration_);
      }
      return time_from_start + step;
    }
  }
  return full_duration_;
}

DynamicSafety::DynamicSafety(
  rclcpp::Node::SharedPtr node)
: DynamicSafety(
    Option().load(
      node->get_node_parameters_interface(),
      node->get_logger()))
{
}

DynamicSafety::DynamicSafety(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: DynamicSafety(
    Option().load(
      node->get_node_parameters_interface(),
      node->get_logger()))
{
}

DynamicSafety::~DynamicSafety()
{
}

DynamicSafety::DynamicSafety(
  const std::shared_ptr<Option> & option)
: impl_ptr_(std::make_unique<Impl>(*option))
{
}

void DynamicSafety::configure(
  const rclcpp::Node::SharedPtr & node)
{
  impl_ptr_->configure(node);
}

void DynamicSafety::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  impl_ptr_->configure(node);
}

void DynamicSafety::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  impl_ptr_->add_trajectory(rt);
}

void DynamicSafety::set_new_trajectory_callback(
  std::function<void(const trajectory_msgs::msg::JointTrajectory::SharedPtr &)> cb)
{
  impl_ptr_->NewTrajectoryCB = cb;
}


void DynamicSafety::update_time(double current_time)
{
  impl_ptr_->update_time(current_time);
}

void DynamicSafety::update_state(const sensor_msgs::msg::JointState::SharedPtr & state)
{
  impl_ptr_->update_state(state);
}

void DynamicSafety::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  impl_ptr_->update_state(joint_names, current_state);
}

void DynamicSafety::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr & state)
{
  impl_ptr_->update_state(joint_names, *state);
}

double DynamicSafety::get_scale()
{
  return impl_ptr_->get_scale();
}

void DynamicSafety::start()
{
  impl_ptr_->start();
}

void DynamicSafety::wait()
{
  impl_ptr_->wait();
}

void DynamicSafety::stop()
{
  impl_ptr_->stop();
}

}  // namespace dynamic_safety
