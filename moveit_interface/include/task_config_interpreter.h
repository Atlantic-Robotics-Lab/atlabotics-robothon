#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stage.h>

#include <map>
#include <string>

namespace mtc = moveit::task_constructor;

class TaskConfigInterpreter
{
public:
    explicit TaskConfigInterpreter(rclcpp::Node* node, const YAML::Node& config);

    bool doTask(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypoints);
    mtc::Task createTask(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypoints);

    mtc::Task task_;  // kept public to match original MoveitInterface public member

    const YAML::Node& config() const { return m_config; }

private:
    rclcpp::Node* m_node;
    YAML::Node m_config;

    void addStagesFromYaml(
        mtc::Task& task,
        const YAML::Node& task_config,
        const std::map<std::string, geometry_msgs::msg::Pose>& named_poses,
        std::map<std::string, mtc::solvers::PlannerInterfacePtr>& planners,
        const std::string& group_name,
        const std::string& hand_frame);
};
