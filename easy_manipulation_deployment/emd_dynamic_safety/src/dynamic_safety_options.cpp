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

#include <fstream>

#include "emd/dynamic_safety/dynamic_safety_options.hpp"
#include "emd/utils.hpp"

namespace dynamic_safety
{

const Option & Option::load(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const rclcpp::Logger & LOGGER)
{
  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    dynamic_parameterization,
    "dynamic_safety.dynamic_parameterization",
    node, LOGGER, true);

  emd::declare_or_get_param<bool>(
    use_description_server,
    "dynamic_safety.use_description_server",
    node, LOGGER, true);

  if (use_description_server) {
    emd::declare_or_get_param<std::string>(
      description_server,
      "dynamic_safety.description_server",
      node, LOGGER);

    using namespace std::chrono_literals;
    auto description_loader_node = std::make_shared<rclcpp::Node>(
      "dynamic_safety_description_loader");
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      description_loader_node, description_server);
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          LOGGER, "Interrupted while waiting for %s service. Exiting.",
          description_server.c_str());
        // TODO(anyone): exception handling.
        break;
      }
      RCLCPP_ERROR(
        LOGGER, "%s service not available, waiting again...",
        description_server.c_str());
    }
    RCLCPP_INFO(LOGGER, "Connected to description server %s!!", description_server.c_str());
    // search and wait for robot_description on param server
    while (robot_description.empty()) {
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto f = parameters_client->get_parameters(
          {"robot_description",
            "robot_description_semantic"});
        RCLCPP_INFO(LOGGER, "Wait for parameters");
        rclcpp::spin_until_future_complete(description_loader_node, f);
        RCLCPP_INFO(LOGGER, "parameter received");
        std::vector<rclcpp::Parameter> values = f.get();
        RCLCPP_INFO(LOGGER, "parameter gotten");
        robot_description = values[0].as_string();
        robot_description_semantic = values[1].as_string();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }

      if (!robot_description.empty()) {
        break;
      } else {
        RCLCPP_ERROR(
          LOGGER, "dynamic_safety is waiting for model"
          " URDF in parameter [robot_description] on the ROS param server.");
      }
      usleep(100000);
    }
    RCLCPP_INFO(
      LOGGER, "Recieved urdf & srdf from param server, parsing...");
  } else {
    emd::declare_or_get_param<std::string>(
      robot_description,
      "dynamic_safety.robot_description",
      node, LOGGER);  // default: "second"
    emd::declare_or_get_param<std::string>(
      robot_description_semantic,
      "dynamic_safety.robot_description_semantic",
      node, LOGGER);  // default: "second"
  }

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    allow_replan,
    "dynamic_safety.allow_replan",
    node, LOGGER, false);

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    benchmark,
    "dynamic_safety.benchmark",
    node, LOGGER, false);

  // Load dyanmic safety parameters
  emd::declare_or_get_param<bool>(
    visualize,
    "dynamic_safety.visualize",
    node, LOGGER, false);

  // -------------- Detailed parameters that needs to set manually --------------

  emd::declare_or_get_param<double>(
    safety_zone_options.look_ahead_time,
    "dynamic_safety.look_ahead_time",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    environment_joint_states_topic,
    "dynamic_safety.environment_joint_states_topic",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    moveit_scene_topic,
    "dynamic_safety.moveit_scene_topic",
    node, LOGGER);
  // Load collision checker parameters
  emd::declare_or_get_param<std::string>(
    collision_checker_options.framework,
    "dynamic_safety.collision_checker.framework",
    node, LOGGER, collision_checker_options.framework);  // default: moveit

  emd::declare_or_get_param<std::string>(
    collision_checker_options.collision_checking_plugin,
    "dynamic_safety.collision_checker.collision_checking_plugin",
    node, LOGGER, collision_checker_options.collision_checking_plugin);  // default: fcl


  // Load collision checker parameters
  emd::declare_or_get_param<bool>(
    collision_checker_options.distance,
    "dynamic_safety.collision_checker.distance",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<bool>(
    collision_checker_options.continuous,
    "dynamic_safety.collision_checker.continuous",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<double>(
    collision_checker_options.step,
    "dynamic_safety.collision_checker.step",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    collision_checker_options.group,
    "dynamic_safety.collision_checker.group",
    node, LOGGER);  // needed for continuous collision checking

  // TODO(anyone): padding

  // -------------- Load overwritable parameters -------------------
  // If the following parameters are defined and greater than zero,
  // dynamic parameterization will not change these parameters
  emd::declare_or_get_param<double>(
    rate,
    "dynamic_safety.rate",
    node, LOGGER, 0);

  emd::declare_or_get_param<double>(
    safety_zone_options.slow_down_time,
    "dynamic_safety.slow_down_time",
    node, LOGGER, 0);

  // Unable to overwrite currently, it is detected by default
  // emd::declare_or_get_param<bool>(
  //   collision_checker_options.realtime,
  //   "dynamic_safety.collision_checker.realtime",
  //   node, LOGGER, true);  // default: true

  emd::declare_or_get_param<int>(
    collision_checker_options.thread_count,
    "dynamic_safety.collision_checker.thread_count",
    node, LOGGER);

  // -------------- Static parameters -------------------
  if (!dynamic_parameterization) {
    // emd::declare_or_get_param<std::string>(
    //   safety_zone_options.unit_type,
    //   "dynamic_safety.safety_zone.unit_type",
    //   node, LOGGER, "second");  // default: "second"

    // // Only second is enabled
    // if (safety_zone_options.unit_type != "second") {
    //   RCLCPP_WARN(
    //     LOGGER, "Wrong safety zone unit type: [%s], default to [second]",
    //     safety_zone_options.unit_type.c_str());
    //   safety_zone_options.unit_type = "second";
    // }
    // TODO(anyone): distance based collision checking
    // I don't think distance makes sense here.

  } else {
    // Dynamically detect realtime OS
    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    collision_checker_options.realtime = false;
    if (realtime_file.is_open()) {
      realtime_file >> collision_checker_options.realtime;
    }

    // Set thread count to max capability
    if (collision_checker_options.thread_count == 0) {
      collision_checker_options.thread_count =
        static_cast<int>(std::thread::hardware_concurrency()) / 2;
    }

    // Joint limit parameters needed for dynamic parameterization
    // of slow down time
    if (safety_zone_options.slow_down_time <= 0) {
      emd::declare_or_get_param<std::string>(
        joint_limits_parameter_server,
        "dynamic_safety.joint_limits_parameter_server",
        node, LOGGER);
      emd::declare_or_get_param<std::string>(
        joint_limits_parameter_namespace,
        "dynamic_safety.joint_limits_parameter_namespace",
        node, LOGGER);

      // Joint Limit loader
      // new node for parameter loading
      auto joint_limits_node = std::make_shared<rclcpp::Node>(
        "dynamic_safety_joint_limits_loader");
      auto joint_limits_parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(
        joint_limits_node, joint_limits_parameter_server);

      while (!joint_limits_parameters_client->wait_for_service()) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            LOGGER, "Interrupted while waiting for %s service. Exiting.",
            joint_limits_parameter_server.c_str());
          // TODO(anyone): exception handling.
          break;
        }
        RCLCPP_ERROR(
          LOGGER, "%s service not available, waiting again...",
          joint_limits_parameter_server.c_str());
      }
      RCLCPP_INFO(
        LOGGER, "Connected to joint limits server %s!!",
        joint_limits_parameter_server.c_str());
      rcl_interfaces::msg::ListParametersResult joint_limits_parameters;
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto joint_limits_future = joint_limits_parameters_client->list_parameters(
          {joint_limits_parameter_namespace},
          5);
        rclcpp::spin_until_future_complete(
          joint_limits_node, joint_limits_future);
        joint_limits_parameters = joint_limits_future.get();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }
      size_t start_idx = joint_limits_parameter_namespace.size() + 1;
      for (auto & name : joint_limits_parameters.prefixes) {
        name.erase(name.begin(), name.begin() + static_cast<int>(start_idx));
        auto jl_f = joint_limits_parameters_client->get_parameters(
          {
            joint_limits_parameter_namespace + "." + name + ".has_velocity_limits",
            joint_limits_parameter_namespace + "." + name + ".max_velocity",
            joint_limits_parameter_namespace + "." + name + ".has_acceleration_limits",
            joint_limits_parameter_namespace + "." + name + ".max_acceleration"
          });
        rclcpp::spin_until_future_complete(joint_limits_node, jl_f);
        auto jl = jl_f.get();
        try {
          if (jl[0].as_bool()) {
            joint_limits[name].first = jl[1].as_double();
          }
          if (jl[2].as_bool()) {
            joint_limits[name].second = jl[3].as_double();
          }
        } catch (const rclcpp::ParameterTypeException & e) {
          RCLCPP_ERROR(LOGGER, e.what());
          continue;
        }
      }
      for (auto & limit : joint_limits) {
        RCLCPP_ERROR(
          LOGGER, "joint: %s max_vel: %f max_accel: %f",
          limit.first.c_str(),
          limit.second.first,
          limit.second.second);
      }
    }
  }

  // Replanner parameters
  if (allow_replan) {
    emd::declare_or_get_param<std::string>(
      replanner_options.framework,
      "dynamic_safety.replanner.framework",
      node, LOGGER, replanner_options.framework);

    emd::declare_or_get_param<std::string>(
      replanner_options.planner,
      "dynamic_safety.replanner.planner",
      node, LOGGER, replanner_options.planner);

    emd::declare_or_get_param<std::string>(
      replanner_options.time_parameterization,
      "dynamic_safety.replanner.time_parameterization",
      node, LOGGER, replanner_options.time_parameterization);

    emd::declare_or_get_param<std::string>(
      replanner_options.group,
      "dynamic_safety.replanner.group",
      node, LOGGER);

    emd::declare_or_get_param<double>(
      replanner_options.deadline,
      "dynamic_safety.replanner.deadline",
      node, LOGGER);

    emd::declare_or_get_param<std::string>(
      replanner_options.joint_limits_parameter_server,
      "dynamic_safety.replanner.joint_limit_parameter_server",
      node, LOGGER, joint_limits_parameter_server);
    emd::declare_or_get_param<std::string>(
      replanner_options.joint_limits_parameter_namespace,
      "dynamic_safety.replanner.joint_limit_parameter_namespace",
      node, LOGGER, joint_limits_parameter_namespace);

    if (replanner_options.framework == "moveit") {
      if (replanner_options.planner == "ompl") {
        emd::declare_or_get_param<std::string>(
          replanner_options.ompl_planner_id,
          "dynamic_safety.replanner.ompl_planner_id",
          node, LOGGER);
      }

      emd::declare_or_get_param<std::string>(
        replanner_options.planner_parameter_server,
        "dynamic_safety.replanner.planner_parameter_server",
        node, LOGGER);
      emd::declare_or_get_param<std::string>(
        replanner_options.planner_parameter_namespace,
        "dynamic_safety.replanner.planner_parameter_namespace",
        node, LOGGER);
    }
  }

  if (visualize) {
    // TODO(Briancbn): scene synchronization
    // emd::declare_or_get_param<bool>(
    //   visualizer_options.publish_scene,
    //   "dynamic_safety.visualize.publish_scene",
    //   node, LOGGER, false);

    emd::declare_or_get_param<double>(
      visualizer_options.publish_frequency,
      "dynamic_safety.visualizer.publish_frequency",
      node, LOGGER, 10);

    emd::declare_or_get_param<double>(
      visualizer_options.step,
      "dynamic_safety.visualizer.step",
      node, LOGGER, 0.1);

    emd::declare_or_get_param<std::string>(
      visualizer_options.topic,
      "dynamic_safety.visualizer.topic",
      node, LOGGER);

    emd::declare_or_get_param<std::string>(
      visualizer_options.tcp_link,
      "dynamic_safety.visualizer.tcp_link",
      node, LOGGER);

    // TODO(Briancbn): scene synchronization
    // emd::declare_or_get_param<std::string>(
    //   visualizer_options.scene_topic,
    //   "dynamic_safety.visualize.scene_topic",
    //   node, LOGGER);
  }

  // Return idiom
  return *this;
}

}  // namespace dynamic_safety
