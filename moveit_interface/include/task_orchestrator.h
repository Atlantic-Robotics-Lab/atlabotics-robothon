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

    // Per-step retry counter. Reset to 0 on index advance or reset().
    // Prevents a permanently-failing step from spinning forever.
    int m_stepRetryCount{0};
    static constexpr int kMaxStepRetries{5};

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

    bool executeGoHome();              // moves to {{home_pose}} named pose (default: home_camera)
    bool executeRetractStylus();       // lifts stylus straight up
    bool executeSingleButtonPress();   // template-driven: reads button_frame from m_stepParams
    bool executeButtonPress();         // perception-driven: calls detect_button service
    bool executeScreenMotion();
    bool executeScreenText();
    bool executeMaze();
    bool executeDropStylus();
    bool executeGrabStylus(TaskType& taskType);
    bool executeCustomTask();

    // ── Phase 5: generic task execution ──────────────────────────────────────
    // Gripper sequence steps — no motion, just gripper open/close.
    bool executeOpenGripper();
    bool executeCloseGripper();

    // Generic pure-motion task driver (Phase 5).
    // Looks up task_name in tasks: YAML, collects tf_frame poses from m_transformedPoses,
    // and calls doTask.  Works for any task that has no gripper/wait stages.
    bool executeGenericMotionTask(const std::string& task_name);
};
