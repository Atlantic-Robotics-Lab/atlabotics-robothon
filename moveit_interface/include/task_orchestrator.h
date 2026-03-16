#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <map>
#include <string>
#include <vector>

#include "types.h"
#include "perception_bridge.h"
#include "gripper_controller.h"
#include "task_config_interpreter.h"

class TaskOrchestrator
{
public:
    explicit TaskOrchestrator(
        rclcpp::Node* node,
        PerceptionBridge* perception,
        GripperController* gripper,
        TaskConfigInterpreter* interpreter);

    void executeTasks(TaskType& task, InterfaceState& out_state);
    bool generateStaticTFPose();
    void reset();

private:
    rclcpp::Node* m_node;
    PerceptionBridge* m_perception;
    GripperController* m_gripper;
    TaskConfigInterpreter* m_interpreter;

    int m_screenTaskCounter{0};
    int m_maxAttempt{2};
    bool m_callShapeService{false};
    bool m_callTextService{false};
    TaskType m_taskType{TaskType::NONE};

    // Helper: calls m_interpreter->doTask (replaces createWaypointTrajectory)
    bool doTask(std::string& task_name, std::map<std::string, geometry_msgs::msg::Pose>& poses);

    bool executeButtonPress();
    bool executeScreenMotion();
    bool executeScreenText();
    bool executeSpeedPress();
    bool executeMaze();
    bool executeDropStylus();
    bool executeGrabStylus(TaskType& taskType);
    bool executeCustomTask();
};
