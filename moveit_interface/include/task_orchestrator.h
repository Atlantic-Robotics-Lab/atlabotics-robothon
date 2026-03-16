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

    // Executes the next step in the YAML task_sequence.
    // task is updated to reflect current TaskType (needed for GRAB_STYLUS variants and BYOD).
    // out_state is set to DONE when all steps complete.
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

    // YAML task_sequence state (Phase 3.6)
    std::vector<TaskSequenceEntry> m_taskSequence;
    int  m_sequenceIndex{0};
    bool m_sequenceLoaded{false};

    // Current step's template params forwarded to doTask (Phase 4.1)
    std::map<std::string, std::string> m_stepParams;

    void loadTaskSequence();
    bool executeStep(const std::string& type,
                     const std::map<std::string, std::string>& params,
                     TaskType& task);

    // Delegates to m_interpreter->doTask, forwarding m_stepParams for template resolution
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
