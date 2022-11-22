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

#ifndef EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_IMPL_HPP_
#define EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_IMPL_HPP_

#include <memory>
#include <string>
#include <utility>
#include <unordered_set>
#include <vector>

#include "emd/dynamic_safety/dynamic_safety.hpp"
#include "emd/dynamic_safety/dynamic_safety_options.hpp"

namespace dynamic_safety
{

class DynamicSafety::Impl
{
public:
  Impl();
  explicit Impl(const Option & option)
  : option_(option), activated_(false)
  {
    // Reset Cache
    env_state_cache_.initRT(sensor_msgs::msg::JointState());
    moveit_scene_cache_.initRT(moveit_msgs::msg::PlanningScene());
    current_state_cache_.initRT(CurrentState());
    current_time_cache_.initRT(0);
    scale_cache_.initRT(1);
  }

  ~Impl()
  {
    stop();
  }

  struct CurrentState
  {
    CurrentState() = default;

    CurrentState(
      const std::vector<std::string> & _joint_names,
      const trajectory_msgs::msg::JointTrajectoryPoint & _state)
    {
      joint_names = _joint_names;
      state = _state;
    }
    std::vector<std::string> joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint state;
  };

  template<typename NodeT>
  void configure(
    const NodeT & node)
  {
    // Initialize flags
    started = false;
    activated_ = false;

    collision_checker_.configure(
      option_.collision_checker_options,
      option_.robot_description,
      option_.robot_description_semantic);

    if (option_.dynamic_parameterization && option_.rate == 0) {
      // Use the new polling function to estimate
      // * Running rate
      option_.rate =
        (2.0 * collision_checker_.polling(option_.safety_zone_options.look_ahead_time));
    }
    RCLCPP_INFO(
      node->get_logger(), "Dynamic safety will run at %dHz.",
      static_cast<int>(option_.rate));

    // Blind zone is the iteration period
    option_.safety_zone_options.collision_checking_deadline = 1.0 / option_.rate;

    if (option_.allow_replan) {
      replanner_.configure(
        option_.replanner_options,
        option_.robot_description,
        option_.robot_description_semantic);
      option_.safety_zone_options.replan_deadline = option_.replanner_options.deadline;
      NewTrajectoryCB = std::bind(
        &DynamicSafety::Impl::add_trajectory, this, std::placeholders::_1);
    }

    if (!safety_zone_.set(option_.safety_zone_options)) {
      throw std::runtime_error("Wrong safety zone parameters");
    } else {
      safety_zone_.print();
    }

    if (option_.visualize) {
      visualizer_.configure(
        option_.visualizer_options,
        option_.safety_zone_options,
        option_.robot_description,
        option_.robot_description_semantic);
    }

    benchmark_stats.clear();

    // Reset Cache
    env_state_cache_.initRT(sensor_msgs::msg::JointState());
    current_time_cache_.initRT(0);
    scale_cache_.initRT(1);

    // double period = 1 / option_.rate;
    env_state_callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions env_state_sub_option;
    env_state_sub_option.callback_group = env_state_callback_group_;
    auto qos = rclcpp::QoS(2);

    if (!option_.environment_joint_states_topic.empty()) {
      env_state_sub_ = node->template create_subscription<sensor_msgs::msg::JointState>(
        option_.environment_joint_states_topic,
        qos,
        [ = ](sensor_msgs::msg::JointState::UniquePtr joint_state_msg) -> void
        {
          if (started) {
            update_state(std::move(joint_state_msg));
          }
        },
        env_state_sub_option
      );
    }

    moveit_scene_callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions moveit_scene_sub_option;
    moveit_scene_sub_option.callback_group = moveit_scene_callback_group_;
    if (!option_.moveit_scene_topic.empty()) {
      moveit_scene_sub_ = node->template create_subscription<moveit_msgs::msg::PlanningScene>(
        option_.moveit_scene_topic,
        qos,
        [ = ](moveit_msgs::msg::PlanningScene::UniquePtr scene_msg) -> void
        {
          if (started) {
            update_scene(std::move(scene_msg));
          }
        },
        env_state_sub_option
      );
    }

    main_callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    main_timer_ = node_->create_wall_timer(
      rclcpp::Duration::from_seconds(1.0 / option_.rate).to_chrono<std::chrono::nanoseconds>(),
      [ = ]() -> void {
        if (started) {
          if (option_.benchmark) {
            if (pf_) {
              pf_->reset();
            }
          }
          _main_loop();
          if (option_.benchmark) {
            if (pf_) {
              pf_->lapse_and_record();
            }
          }
        }
      },
      main_callback_group_);
  }

  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

  void update_time(double current_time);

  void update_state(const sensor_msgs::msg::JointState::SharedPtr & state);

  void update_state(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_state);

  void update_scene(const moveit_msgs::msg::PlanningScene::SharedPtr & scene_msg);

  double get_scale();

  void start();

  void wait();

  void stop();

  std::function<void(const trajectory_msgs::msg::JointTrajectory::SharedPtr &)> NewTrajectoryCB;

protected:
  void _configure();

  void _deadline_cb(rclcpp::QOSDeadlineRequestedInfo &);

  void _main_loop();

  double _cal_scale_time(
    const CurrentState & current_state,
    double current_scale,
    double target_scale);

  void _handle_replanner(double start_state_time);

  // Temporary functions to be moved into collision checker
  double _back_track_last_collision();
  double full_duration_;
  // Temporary map better handling needed
  std::unordered_set<std::string> joint_names;

  Option option_;

  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr env_state_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr moveit_scene_sub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::CallbackGroup::SharedPtr env_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr moveit_scene_callback_group_;
  rclcpp::CallbackGroup::SharedPtr main_callback_group_;

  CollisionChecker collision_checker_;
  SafetyZone safety_zone_;
  // NextPointPublisher next_point_publisher_;
  Replanner replanner_;
  Visualizer visualizer_;

  double collision_time_point_;
  // double replan_time_point_;

  // uint8_t zone;

  std::atomic_bool activated_;
  std::atomic_bool started;

  std::vector<double> benchmark_stats;

  std::promise<void> sig_;
  std::future<void> future_;

  emd::TimeProfiler<> * pf_;

  // realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectoryPoint> state_cache_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState> env_state_cache_;
  realtime_tools::RealtimeBuffer<moveit_msgs::msg::PlanningScene> moveit_scene_cache_;
  realtime_tools::RealtimeBuffer<CurrentState> current_state_cache_;
  realtime_tools::RealtimeBuffer<double> current_time_cache_;
  realtime_tools::RealtimeBuffer<double> scale_cache_;
  // realtime_tools::RealtimeBuffer<octomap::OcTree> env_state_cache_;
};


}  // namespace dynamic_safety


#endif  // EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_IMPL_HPP_
