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

#include <memory>
#include <string>
#include <vector>

#include "emd/dynamic_safety/dynamic_safety_trajectory_controller.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace dynamic_safety
{
controller_interface::InterfaceConfiguration
DynamicSafetyTrajectoryController::state_interface_configuration() const
{
  return JointTrajectoryController::state_interface_configuration();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamicSafetyTrajectoryController::on_configure(const rclcpp_lifecycle::State & state)
{
  auto result = JointTrajectoryController::on_configure(state);
  if (result ==
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    // Load safety officer configuration
    safety_officer_ = std::make_unique<DynamicSafety>(get_node());

    // Remove existing subscriber and action monitor setup
    joint_command_subscriber_.reset();
    action_server_.reset();

    // Create new subscriber callback
    auto sub_callback =
      [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg) -> void {
        if (!validate_trajectory_msg(*msg)) {
          return;
        }

        // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
        // always replace old msg with new one for now
        if (subscriber_is_active_) {
          this->add_new_trajectory_msg(msg);
        }
      };

    joint_command_subscriber_ =
      get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), sub_callback);

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
      get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
      std::string(get_node()->get_name()) + "/follow_joint_trajectory",
      std::bind(&DynamicSafetyTrajectoryController::goal_received_callback, this, _1, _2),
      std::bind(&DynamicSafetyTrajectoryController::goal_cancelled_callback, this, _1),
      std::bind(&DynamicSafetyTrajectoryController::goal_accepted_callback, this, _1));

    safety_officer_->configure(get_node());
    safety_officer_->set_new_trajectory_callback(
      std::bind(
        &DynamicSafetyTrajectoryController::add_new_trajectory_msg, this,
        std::placeholders::_1));
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DynamicSafetyTrajectoryController::on_activate(const rclcpp_lifecycle::State & state)
{
  TimeData time_data;
  time_data.time = get_node()->now();
  time_data.period = rclcpp::Duration(0, 0);
  time_data.uptime = get_node()->now();
  time_data_.initRT(time_data);
  scaling_factor_.initRT(1.0);
  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type DynamicSafetyTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }

  auto compute_error_for_joint = [&](
    JointTrajectoryPoint & error, int index,
    const JointTrajectoryPoint & current,
    const JointTrajectoryPoint & desired)
    {
      // error defined as the difference between current and desired
      error.positions[index] =
        angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
      if (has_velocity_state_interface_ && has_velocity_command_interface_) {
        error.velocities[index] = desired.velocities[index] - current.velocities[index];
      }
      if (has_acceleration_state_interface_ && has_acceleration_command_interface_) {
        error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
      }
    };

  // Main Speed scaling difference...
  // Adjust time with scaling factor
  TimeData time_data;
  time_data.time = time;
  double scale = safety_officer_->get_scale();
  scaling_factor_.writeFromNonRT(scale);
  time_data.period = period * (*scaling_factor_.readFromRT());
  time_data.uptime = time_data_.readFromRT()->uptime + time_data.period;
  rclcpp::Time traj_time = time_data_.readFromRT()->uptime + time_data.period;
  time_data_.writeFromNonRT(time_data);

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    (*new_external_msg)->header.stamp = time_data.uptime;
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&](auto & joint_interface, const std::vector<double> & trajectory_point_interface)
    {
      for (size_t index = 0; index < dof_; ++index) {
        joint_interface[index].get().set_value(trajectory_point_interface[index]);
      }
    };

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_hardware(state_current_);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg()) {
    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already()) {
      first_sample = true;
      if (params_.open_loop_control) {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, last_commanded_state_);
      } else {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_);
      }

      safety_officer_->start();
    }
    double current_time =
      (traj_time - (*traj_point_active_ptr_)->get_trajectory_start_time()).seconds();
    safety_officer_->update_time(current_time);
    safety_officer_->update_state(params_.joints, state_current_);

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point =
      (*traj_point_active_ptr_)
      ->sample(time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

    if (valid_point) {
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      double time_difference = 0.0;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index) {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        if (
          (before_last_point || first_sample) &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.state_tolerance[index], false))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.goal_state_tolerance[index], false))
        {
          outside_goal_tolerance = true;

          if (default_tolerances_.goal_time_tolerance != 0.0) {
            // if we exceed goal_time_tolerance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            time_difference = get_node()->now().seconds() - traj_end.seconds();

            if (time_difference > default_tolerances_.goal_time_tolerance) {
              within_goal_time = false;
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time) {
        if (use_closed_loop_pid_adapter_) {
          // Update PIDs
          for (auto i = 0ul; i < dof_; ++i) {
            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
              pids_[i]->computeCommand(
              state_desired_.positions[i] - state_current_.positions[i],
              state_desired_.velocities[i] - state_current_.velocities[i],
              (uint64_t)period.nanoseconds());
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_) {
          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
        }
        if (has_velocity_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          } else {
            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
          }
        }
        if (has_acceleration_command_interface_) {
          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
        }
        if (has_effort_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
          } else {
            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
          }
        }

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }

      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving) {
          set_hold_position();
          auto result = std::make_shared<FollowJTrajAction::Result>();

          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

          // check goal tolerance
        } else if (!before_last_point) {
          if (!outside_goal_tolerance) {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            safety_officer_->stop();
            active_goal->setSucceeded(res);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

            RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
          } else if (!within_goal_time) {
            set_hold_position();
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            safety_officer_->stop();
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            RCLCPP_WARN(
              get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
              time_difference);
          }
          // else, run another cycle while waiting for outside_goal_tolerance
          // to be satisfied or violated within the goal_time_tolerance
        }
      } else if (tolerance_violated_while_moving) {
        set_hold_position();
        RCLCPP_ERROR(get_node()->get_logger(), "Holding position due to state tolerance violation");
      }
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

void DynamicSafetyTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    this->add_new_trajectory_msg(traj_msg);  // This would be overriden
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::seconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void DynamicSafetyTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  safety_officer_->add_trajectory(traj_msg);
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}


}  // namespace dynamic_safety

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dynamic_safety::DynamicSafetyTrajectoryController,
  controller_interface::ControllerInterface)
