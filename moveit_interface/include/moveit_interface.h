#pragma once

#include <memory>
#include <thread>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "std_srvs/srv/trigger.hpp"

#include "types.h"
#include "gripper_controller.h"
#include "perception_bridge.h"
#include "task_config_interpreter.h"
#include "task_orchestrator.h"

using moveit::planning_interface::MoveGroupInterface;

class MoveitInterface : public rclcpp::Node
{
public:
    explicit MoveitInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MoveitInterface();

    void run();
    void setupPlanningScene();

    std::shared_ptr<MoveGroupInterface> m_movegroupInterface;
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface scene_;

private:
    void setParams();
    void triggerTaskCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    YAML::Node m_config;
    std::string m_yamlPath;
    double m_maxVel{0.0};
    double m_maxAcc{0.0};
    int m_srvAttempts{0};

    bool m_completed{false};

    InterfaceState m_state{InterfaceState::IDLE};
    TaskType m_taskType{TaskType::NONE};
    TaskType m_nextTaskType{TaskType::NONE};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_triggerTask;

    // Sub-components (constructed in constructor body after node is ready)
    std::unique_ptr<GripperController>      m_gripper;
    std::unique_ptr<PerceptionBridge>       m_perception;
    std::unique_ptr<TaskConfigInterpreter>  m_interpreter;
    std::unique_ptr<TaskOrchestrator>       m_orchestrator;
};
