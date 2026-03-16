#include "task_orchestrator.h"

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

TaskOrchestrator::TaskOrchestrator(
    rclcpp::Node* node,
    PerceptionBridge* perception,
    GripperController* gripper,
    TaskConfigInterpreter* interpreter)
    : m_node(node)
    , m_perception(perception)
    , m_gripper(gripper)
    , m_interpreter(interpreter)
{
}

// Forwards m_stepParams for template variable resolution (Phase 4.1)
bool TaskOrchestrator::doTask(std::string& task_name, std::map<std::string, geometry_msgs::msg::Pose>& poses)
{
    return m_interpreter->doTask(task_name, poses, m_stepParams);
}

void TaskOrchestrator::reset()
{
    m_screenTaskCounter = 0;
    m_callShapeService  = false;
    m_callTextService   = false;
    m_sequenceIndex     = 0;
    m_sequenceLoaded    = false;
    m_stepParams.clear();
    m_perception->resetState();
}

bool TaskOrchestrator::generateStaticTFPose()
{
    return m_perception->generateStaticTFPose();
}

// ─── loadTaskSequence (Phase 3.6) ─────────────────────────────────────────────
// Reads task_sequence from YAML and populates m_taskSequence.
void TaskOrchestrator::loadTaskSequence()
{
    m_taskSequence.clear();
    const auto& seq = m_interpreter->config()["task_sequence"];
    if (!seq || !seq.IsSequence()) {
        RCLCPP_WARN(m_node->get_logger(),
                    "No task_sequence found in config; sequence will be empty");
        return;
    }
    for (const auto& entry : seq) {
        if (!entry["type"]) continue;
        TaskSequenceEntry e;
        e.type = entry["type"].as<std::string>();
        if (entry["params"] && entry["params"].IsMap()) {
            for (const auto& kv : entry["params"])
                e.params[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
        m_taskSequence.push_back(e);
    }
    RCLCPP_INFO(m_node->get_logger(), "Loaded %zu task sequence steps", m_taskSequence.size());
}

// ─── executeStep (Phase 3.6) ──────────────────────────────────────────────────
// Dispatches a single task_sequence step to the appropriate execute* method.
// Sets m_stepParams so template vars reach doTask (Phase 4.1).
bool TaskOrchestrator::executeStep(
    const std::string& type,
    const std::map<std::string, std::string>& params,
    TaskType& task)
{
    m_stepParams = params;  // make params available inside execute* → doTask chain

    if (type == "speed_press")    return executeSpeedPress();
    if (type == "press_buttons")  return executeButtonPress();
    if (type == "grab_stylus_touch") {
        task = TaskType::GRAB_STYLUS_TOUCH;
        return executeGrabStylus(task);
    }
    if (type == "grab_stylus_magnet") {
        task = TaskType::GRAB_STYLUS_MAGNET;
        return executeGrabStylus(task);
    }
    if (type == "screen_shape") return executeScreenMotion();
    if (type == "screen_text")  return executeScreenText();
    if (type == "maze")         return executeMaze();
    if (type == "drop_stylus")  return executeDropStylus();

    RCLCPP_ERROR(m_node->get_logger(), "Unknown task_sequence type: '%s' — skipping",
                 type.c_str());
    return true;  // unknown step treated as done to avoid infinite retry
}

// ─── executeTasks (Phase 3.6) ─────────────────────────────────────────────────
// Replaces the hardcoded switch-case with a YAML-driven sequence.
void TaskOrchestrator::executeTasks(TaskType& task, InterfaceState& out_state)
{
    // BYOD mode: custom_task:true bypasses the standard sequence
    if (task == TaskType::BYOD) {
        bool done = executeCustomTask();
        if (done) out_state = InterfaceState::DONE;
        return;
    }

    // Load sequence from YAML on first call (or after reset)
    if (!m_sequenceLoaded) {
        loadTaskSequence();
        m_sequenceLoaded = true;
    }

    // All steps done
    if (m_sequenceIndex >= static_cast<int>(m_taskSequence.size())) {
        reset();
        out_state = InterfaceState::DONE;
        return;
    }

    const TaskSequenceEntry& entry = m_taskSequence[m_sequenceIndex];
    bool step_done = executeStep(entry.type, entry.params, task);
    if (step_done) {
        RCLCPP_INFO(m_node->get_logger(), "Step [%d/%zu] '%s' completed",
                    m_sequenceIndex + 1, m_taskSequence.size(), entry.type.c_str());
        m_sequenceIndex++;
    }
}

// ─── execute* implementations (verbatim from Phase 2) ─────────────────────────

bool TaskOrchestrator::executeMaze()
{
    std::string target_frame = "base_link";
    std::string source_frame = "blue_button";
    std::string current_task = "solve_maze";

    try
    {
        geometry_msgs::msg::TransformStamped tfstamped;
        tfstamped.transform.translation.x = m_perception->m_transformedPoses[source_frame].position.x;
        tfstamped.transform.translation.y = m_perception->m_transformedPoses[source_frame].position.y;
        tfstamped.transform.translation.z = m_perception->m_transformedPoses[source_frame].position.z;
        tfstamped.transform.rotation.x = m_perception->m_transformedPoses[source_frame].orientation.x;
        tfstamped.transform.rotation.y = m_perception->m_transformedPoses[source_frame].orientation.y;
        tfstamped.transform.rotation.z = m_perception->m_transformedPoses[source_frame].orientation.z;
        tfstamped.transform.rotation.w = m_perception->m_transformedPoses[source_frame].orientation.w;

        RCLCPP_INFO(m_node->get_logger(), "m_mazePath size: %zu", m_perception->m_mazePath.size());

        std::map<std::string, geometry_msgs::msg::Pose> named_poses;
        named_poses["screen_align_pose"] = m_perception->m_transformedPoses["align_frame"];
        int index = 10;
        for (auto pose : m_perception->m_mazePath)
        {
            geometry_msgs::msg::Pose transformed_maze_pose;
            tf2::doTransform(pose, transformed_maze_pose, tfstamped);
            transformed_maze_pose.orientation = m_perception->m_transformedPoses["align_frame"].orientation;
            std::string name = "maze_pathpoint_" + std::to_string(index);
            RCLCPP_INFO(m_node->get_logger(), "nameeeee %s", name.c_str());
            named_poses[name] = transformed_maze_pose;
            index++;
        }

        bool validTrajectory = false;
        validTrajectory = doTask(current_task, named_poses);
        if (validTrajectory)
        {
            current_task = "retract_maze";
            validTrajectory = doTask(current_task, named_poses);
            return validTrajectory;
        }
        else
        {
            RCLCPP_INFO(m_node->get_logger(), "Could not execute maze task");
            return false;
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(m_node->get_logger(), "%s", e.what());
        return false;
    }
}

bool TaskOrchestrator::executeDropStylus()
{
    std::string current_task = "place_stylus";

    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    named_poses["stylus_pose"] = m_perception->m_transformedPoses["stylus"];

    bool validTrajectory = false;
    validTrajectory = doTask(current_task, named_poses);

    if (validTrajectory)
    {
        bool gripper_state = false;
        m_gripper->gripperService(gripper_state);
        current_task = "retract_stylus";
        validTrajectory = doTask(current_task, named_poses);
        current_task = "home_pose";
        validTrajectory = doTask(current_task, named_poses);
        return validTrajectory;
    }
    else
    {
        RCLCPP_INFO(m_node->get_logger(), "Could not reach stylus pose");
        return false;
    }
}

bool TaskOrchestrator::executeGrabStylus(TaskType& taskType)
{
    std::string current_task = "";
    if (taskType == TaskType::GRAB_STYLUS_TOUCH)
    {
        bool gripper_state = false;
        m_gripper->gripperService(gripper_state); // Close gripper

        current_task = "pick_stylus";

        std::map<std::string, geometry_msgs::msg::Pose> named_poses;
        named_poses["stylus_pose"] = m_perception->m_transformedPoses["stylus"];

        bool validTrajectory = false;
        validTrajectory = doTask(current_task, named_poses);

        if (validTrajectory)
        {
            gripper_state = true;
            m_gripper->gripperService(gripper_state);
            sleep(1);
            RCLCPP_INFO(m_node->get_logger(), "Waiting done");
            current_task = "retract_stylus";
            validTrajectory = doTask(current_task, named_poses);
            return validTrajectory;
        }
        else
        {
            RCLCPP_INFO(m_node->get_logger(), "Could not reach stylus pose");
            return false;
        }
    }
    else if (taskType == TaskType::GRAB_STYLUS_MAGNET)
    {
        current_task = "retract_stylus_invert";
        geometry_msgs::msg::TransformStamped tfstamped = m_perception->getTfBuffer().lookupTransform("base_link", "stylus_calibration", m_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
        geometry_msgs::msg::Pose calibPose;
        calibPose.position.x = tfstamped.transform.translation.x;
        calibPose.position.y = tfstamped.transform.translation.y;
        calibPose.position.z = tfstamped.transform.translation.z;
        calibPose.orientation.x = tfstamped.transform.rotation.x;
        calibPose.orientation.y = tfstamped.transform.rotation.y;
        calibPose.orientation.z = tfstamped.transform.rotation.z;
        calibPose.orientation.w = tfstamped.transform.rotation.w;

        std::map<std::string, geometry_msgs::msg::Pose> named_poses;
        named_poses["stylus_calibration_precise"] = calibPose;

        bool validTrajectory = false;
        validTrajectory = doTask(current_task, named_poses);
        return validTrajectory;
    }

    return false;
}

bool TaskOrchestrator::executeScreenText()
{
    std::string target_frame = "";
    std::string source_frame = "";
    std::string current_task = "";

    {
        if (!m_perception->m_service_map["detect_text"].srv_response.success)
        {
            m_perception->callTriggerService("detect_text");
            return false;
        }

        if (m_perception->m_screenCommand == "")
        {
            RCLCPP_INFO(m_node->get_logger(), "Waiting for m_screenCommand");
            return false;
        }

        current_task = "screen_text";
        target_frame = "base_link";

        ParsedTask parsedTask;
        m_perception->parseTaskCommand(m_perception->m_screenCommand, parsedTask);
        std::map<std::string, geometry_msgs::msg::Pose> named_poses;

        RCLCPP_INFO(m_node->get_logger(), "ID: %d size %zu", parsedTask.id, parsedTask.task_names.size());
        for (const auto& name : parsedTask.task_names)
        {
            RCLCPP_INFO(m_node->get_logger(), "Task from screen: %s", name.c_str());
            source_frame = name;
            geometry_msgs::msg::Pose targetPose;
            try
            {
                targetPose = m_perception->m_transformedPoses[name];
                named_poses[name] = targetPose;
                RCLCPP_INFO(m_node->get_logger(), "  pose: %.3f %.3f", targetPose.position.x, targetPose.position.y);
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_INFO(m_node->get_logger(), "Could not transform 'base_link' to 'point on screen: %s", ex.what());
                return false;
            }
        }

        named_poses["screen"] = m_perception->m_transformedPoses["screen"];

        if (m_interpreter->config()["planning"].as<bool>())
        {
            bool validTrajectory = false;
            validTrajectory = doTask(current_task, named_poses);
            if (validTrajectory && m_screenTaskCounter < 2)
            {
                m_screenTaskCounter++;
                m_perception->m_service_map["detect_text"].srv_response.success = false;
                return false;
            }
            else
            {
                current_task = "retract_align_screen";
                bool homepose = doTask(current_task, named_poses);
                m_callTextService = false;
                m_taskType = TaskType::END;
                return true;
            }
        }
    }
    return false;
}

bool TaskOrchestrator::executeScreenMotion()
{
    std::string target_frame = "";
    std::string source_frame = "";
    std::string current_task = "";

    RCLCPP_INFO(m_node->get_logger(), "m_callShapeService: %d", m_callShapeService);
    if (!m_callShapeService)
    {
        current_task = "screen_approach";
        source_frame = "align_frame";

        std::map<std::string, geometry_msgs::msg::Pose> named_poses;
        named_poses["screen_align_pose"] = m_perception->m_transformedPoses["align_frame"];
        named_poses["screen"] = m_perception->m_transformedPoses["screen"];

        bool validTrajectory = false;
        validTrajectory = doTask(current_task, named_poses);
        m_callShapeService = validTrajectory;
        return false;
    }
    else
    {
        RCLCPP_INFO(m_node->get_logger(), "m_labelData: %s", m_perception->m_labelData.c_str());
        if (!m_perception->m_service_map["detect_shape"].srv_response.success)
        {
            m_perception->callTriggerService("detect_shape");
            return false;
        }

        if (m_perception->m_shapePoses.poses.empty())
        {
            RCLCPP_INFO(m_node->get_logger(), "Waiting for detection");
            return false;
        }

        RCLCPP_INFO(m_node->get_logger(), "Shape Detected");
        current_task = "screen_draw";
        target_frame = "base_link";
        source_frame = m_perception->m_shapePoses.header.frame_id;

        try
        {
            geometry_msgs::msg::TransformStamped tfstamped = m_perception->getTfBuffer().lookupTransform(target_frame, source_frame, m_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));

            std::map<std::string, geometry_msgs::msg::Pose> named_poses;
            named_poses["background"] = m_perception->m_transformedPoses["Background"];
            int index = 0;
            for (auto pose : m_perception->m_shapePoses.poses)
            {
                geometry_msgs::msg::Pose temp;
                m_perception->getTf(pose, temp, tfstamped, target_frame, source_frame);
                RCLCPP_DEBUG(m_node->get_logger(), " pose %.3f %.3f  temp %.3f %.3f",
                    pose.position.x, pose.position.y, temp.position.x, temp.position.y);
                temp.position.z = m_perception->m_transformedPoses["Background"].position.z;
                std::string name = "screen_draw" + std::to_string(index);
                named_poses[name] = temp;
                index++;
            }

            named_poses["screen"] = m_perception->m_transformedPoses["screen"];

            bool validTrajectory = false;
            validTrajectory = doTask(current_task, named_poses);
            if (validTrajectory && m_screenTaskCounter < m_maxAttempt)
            {
                m_screenTaskCounter++;
                m_perception->m_service_map["detect_shape"].srv_response.success = false;
                return false;
            }
            else
            {
                m_screenTaskCounter = 0;
                m_callShapeService = false;
                m_taskType = TaskType::END;
                return true;
            }
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(m_node->get_logger(), "Could not transform 'base_link' to 'camera': %s", ex.what());
            return false;
        }
    }
}

bool TaskOrchestrator::executeSpeedPress()
{
    RCLCPP_INFO(m_node->get_logger(), "executeSpeedPress");

    std::string current_task = "speed_test";

    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    named_poses["blue_button"] = m_perception->m_transformedPoses["blue_button"];
    named_poses["red_button"]  = m_perception->m_transformedPoses["red_button"];

    bool validTrajectory = false;
    validTrajectory = doTask(current_task, named_poses);
    m_taskType = TaskType::END;
    return validTrajectory;
}

bool TaskOrchestrator::executeButtonPress()
{
    std::string current_task = "";
    if (m_perception->m_service_map["detect_button"].srv_response.success)
    {
        if (m_perception->m_buttonStatus != "None")
        {
            std::map<std::string, geometry_msgs::msg::Pose> named_poses;
            if (m_perception->m_buttonStatus == "Red")
            {
                current_task = "press_red_button";
                named_poses["red_button"] = m_perception->m_transformedPoses["red_button"];
            }
            else
            {
                current_task = "press_blue_button";
                named_poses["blue_button"] = m_perception->m_transformedPoses["blue_button"];
            }

            bool validTrajectory = false;
            validTrajectory = doTask(current_task, named_poses);
            m_perception->m_service_map["detect_button"].srv_response.success = false;
            m_taskType = TaskType::END;
            return validTrajectory;
        }
        else
        {
            return false;
        }
    }
    else
    {
        m_perception->callTriggerService("detect_button");
        return false;
    }
}

bool TaskOrchestrator::executeCustomTask()
{
    std::string target_frame = "";
    std::string source_frame = "";
    std::string current_task = "";

    {
        RCLCPP_INFO(m_node->get_logger(), "color_sort service response: %d", m_perception->m_service_map["color_sort"].srv_response.success);
        if (!m_perception->m_service_map["color_sort"].srv_response.success)
        {
            m_perception->callTriggerService("color_sort");
            return false;
        }

        if (m_perception->m_detectionPoses.empty())
        {
            RCLCPP_INFO(m_node->get_logger(), "Waiting for detection");
            return false;
        }

        if (m_perception->m_screenCommand == "")
        {
            RCLCPP_INFO(m_node->get_logger(), "Waiting for m_screenCommand");
            return false;
        }

        current_task = m_perception->m_screenCommand; // Input from topic
        target_frame = "base_link";
        source_frame = "camera_color_optical_frame";

        std::map<std::string, std::vector<geometry_msgs::msg::Pose>> sortingPoses;

        try
        {
            RCLCPP_INFO(m_node->get_logger(), "%d ", (int)m_perception->m_detectionPoses.size());
            geometry_msgs::msg::TransformStamped tfstamped = m_perception->getTfBuffer().lookupTransform(target_frame, source_frame, m_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
            geometry_msgs::msg::TransformStamped tfstamped_gripper = m_perception->getTfBuffer().lookupTransform(target_frame, "ee_touch", m_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));

            std::map<std::string, geometry_msgs::msg::Pose> named_poses;
            for (auto detection_array : m_perception->m_detectionPoses)
            {
                std::string name = detection_array.header.frame_id + "_object";
                for (auto pose : detection_array.poses)
                {
                    geometry_msgs::msg::Pose objPose;
                    m_perception->getTf(pose, objPose, tfstamped, target_frame, source_frame);
                    objPose.orientation = tfstamped_gripper.transform.rotation;
                    sortingPoses[name].push_back(objPose);
                }
            }

            RCLCPP_INFO(m_node->get_logger(), "%d ", (int)sortingPoses.size());

            for (const auto& [bin, poses] : sortingPoses) {
                RCLCPP_INFO(m_node->get_logger(), "Bin: %s", bin.c_str());
                for (const auto& pose : poses) {
                    RCLCPP_INFO(m_node->get_logger(), "  x: %.2f y: %.2f z: %.2f",
                        pose.position.x, pose.position.y, pose.position.z);
                }
            }

            for (const auto& [key, pose_vector] : sortingPoses) {
                if (!pose_vector.empty()) {
                    RCLCPP_INFO(m_node->get_logger(), "key: %s", key.c_str());
                    named_poses[key] = pose_vector[0];
                }
            }

            bool validTrajectory = false;
            validTrajectory = doTask(current_task, named_poses);
            m_perception->m_service_map["color_sort"].srv_response.success = false;
            return validTrajectory;
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(m_node->get_logger(), "Could not transform 'base_link' to 'camera': %s", ex.what());
            return false;
        }
    }
}
