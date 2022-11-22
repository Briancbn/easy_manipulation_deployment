// Copyright 2022 ROS Industrial Consortium Asia Pacific
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
#include <unordered_map>
#include <utility>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "emd/dynamic_safety/safety_zone.hpp"
#include "emd/dynamic_safety/collision_checker.hpp"
#include "emd/dynamic_safety/replanner.hpp"
#include "emd/dynamic_safety/visualizer.hpp"

#ifndef EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_OPTIONS_HPP_
#define EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_OPTIONS_HPP_

namespace dynamic_safety
{

struct Option : std::enable_shared_from_this<Option>
{
  double rate;

  bool dynamic_parameterization;

  bool use_description_server;

  std::string description_server;

  std::string joint_limits_parameter_server;
  std::string joint_limits_parameter_namespace;

  std::unordered_map<std::string, std::pair<double, double>> joint_limits;

  std::string robot_description;
  std::string robot_description_semantic;

  std::string environment_joint_states_topic;

  std::string moveit_scene_topic;

  bool allow_replan;

  bool benchmark;

  bool visualize;

  SafetyZone::Option safety_zone_options;

  CollisionCheckerOption collision_checker_options;

  // NextPointPublisher::Option next_point_publisher_options;

  ReplannerOption replanner_options;

  Visualizer::Option visualizer_options;

  std::shared_ptr<Option> load(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
    const rclcpp::Logger & LOGGER = rclcpp::get_logger("dynamic_safety"));
};

}  // namespace dynamic_safety


#endif  // EMD__DYNAMIC_SAFETY__DYNAMIC_SAFETY_OPTIONS_HPP_
