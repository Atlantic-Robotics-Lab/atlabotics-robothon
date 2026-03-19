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

    // params: optional template variables resolved into {{variable}} placeholders (Phase 4.1)
    bool doTask(std::string& current_task,
                std::map<std::string, geometry_msgs::msg::Pose>& waypoints,
                const std::map<std::string, std::string>& params = {});

    mtc::Task createTask(std::string& current_task,
                         std::map<std::string, geometry_msgs::msg::Pose>& waypoints,
                         const std::map<std::string, std::string>& params = {});

    mtc::Task task_;  // kept public to match original MoveitInterface public member

    const YAML::Node& config() const { return m_config; }

    // Replaces {{key}} with params[key] in s. Returns s unchanged if no placeholders. (Phase 4.1)
    // Public so TaskOrchestrator can resolve template vars when collecting stage poses.
    static std::string resolveTemplateVar(const std::string& s,
                                          const std::map<std::string, std::string>& params);

private:
    rclcpp::Node* m_node;
    YAML::Node m_config;

    void addStagesFromYaml(
        mtc::Task& task,
        const YAML::Node& task_config,
        const std::map<std::string, geometry_msgs::msg::Pose>& named_poses,
        std::map<std::string, mtc::solvers::PlannerInterfacePtr>& planners,
        const std::string& group_name,
        const std::string& hand_frame,
        const std::map<std::string, std::string>& params = {});
};
