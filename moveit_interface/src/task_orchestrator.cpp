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
    m_stepRetryCount    = 0;
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

    if (type == "localize_board") return executeLocalizeBoard();
    if (type == "go_home")        return executeGoHome();
    if (type == "press_button")   return executeSingleButtonPress();
    if (type == "press_buttons")  return executeButtonPress();
    if (type == "retract_stylus") return executeRetractStylus();
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

    // ── Phase 5: gripper sequence steps ──────────────────────────────────────
    if (type == "open_gripper")  return executeOpenGripper();
    if (type == "close_gripper") return executeCloseGripper();

    // ── Phase 5: generic fallthrough ─────────────────────────────────────────
    // Any type not handled above is looked up in tasks: YAML.
    // Works for pure-motion task templates (no gripper/wait stages).
    // To add a new task: define it in tasks: YAML — no C++ changes needed.
    return executeGenericMotionTask(type);
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
        m_stepRetryCount = 0;
    } else {
        m_stepRetryCount++;
        RCLCPP_DEBUG(m_node->get_logger(), "Step [%d/%zu] '%s' not done (attempt %d)",
                     m_sequenceIndex + 1, m_taskSequence.size(),
                     entry.type.c_str(), m_stepRetryCount);
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
    // stylus_frame param selects which TF frame is the place target.
    std::string stylus_tf = m_stepParams.count("stylus_frame") ? m_stepParams.at("stylus_frame") : "stylus";

    std::string current_task = "place_stylus";

    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    named_poses[stylus_tf] = m_perception->m_transformedPoses[stylus_tf];

    bool validTrajectory = doTask(current_task, named_poses);

    if (!validTrajectory)
        RCLCPP_INFO(m_node->get_logger(), "Could not reach stylus pose");
    // Gripper open/close are separate task_sequence steps — not embedded here.
    return validTrajectory;
}

bool TaskOrchestrator::executeGrabStylus(TaskType& taskType)
{
    // stylus_frame param selects which TF frame to use as the pick target.
    std::string stylus_tf = m_stepParams.count("stylus_frame") ? m_stepParams.at("stylus_frame") : "stylus";
    std::string current_task = "";

    if (taskType == TaskType::GRAB_STYLUS_TOUCH)
    {
        current_task = "pick_stylus";

        std::map<std::string, geometry_msgs::msg::Pose> named_poses;
        named_poses[stylus_tf] = m_perception->m_transformedPoses[stylus_tf];

        bool validTrajectory = doTask(current_task, named_poses);

        if (!validTrajectory)
            RCLCPP_INFO(m_node->get_logger(), "Could not reach stylus pose");
        // Gripper open/close are separate task_sequence steps — not embedded here.
        return validTrajectory;
    }
    else if (taskType == TaskType::GRAB_STYLUS_MAGNET)
    {
        // stylus_calib_frame param selects the TF calibration frame (default: stylus_calibration).
        std::string calib_tf = m_stepParams.count("stylus_calib_frame") ? m_stepParams.at("stylus_calib_frame") : "stylus_calibration";

        current_task = "retract_stylus_invert";
        geometry_msgs::msg::TransformStamped tfstamped = m_perception->getTfBuffer().lookupTransform("base_link", calib_tf, m_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
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
                doTask(current_task, named_poses);
                m_callTextService = false;
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
                return true;
            }
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(m_node->get_logger(), "Could not transform 'base_link' to 'camera': %s", ex.what());
            return false;
        }
    }
}

// ─── Phase 5: gripper sequence steps ──────────────────────────────────────────

bool TaskOrchestrator::executeOpenGripper()
{
    RCLCPP_INFO(m_node->get_logger(), "executeOpenGripper");
    bool state = true;
    m_gripper->gripperService(state);
    // gripperService is async — sleep long enough for the gripper to physically move
    // before the next sequence step starts planning/executing.
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

bool TaskOrchestrator::executeCloseGripper()
{
    RCLCPP_INFO(m_node->get_logger(), "executeCloseGripper");
    bool state = false;
    m_gripper->gripperService(state);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

// ─── Phase 5: generic pure-motion task driver ─────────────────────────────────
// Looks up task_name in tasks: YAML. Collects all tf_frame targets from the task's
// stages (resolving template vars from m_stepParams), then calls doTask.
// Works for any task template that contains only move_to / move_relative / move_to_path
// stages — no gripper or wait stages.
bool TaskOrchestrator::executeGenericMotionTask(const std::string& task_name)
{
    const YAML::Node& task_node = m_interpreter->config()["tasks"][task_name];
    if (!task_node) {
        RCLCPP_ERROR(m_node->get_logger(),
                     "Unknown task type '%s' — not found in tasks: config, skipping",
                     task_name.c_str());
        return true;  // skip to avoid infinite retry
    }

    RCLCPP_INFO(m_node->get_logger(), "executeGenericMotionTask: %s", task_name.c_str());

    // Walk the stage list and collect every tf_frame target into named_poses.
    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    if (task_node["stages"]) {
        for (const auto& stage : task_node["stages"]) {
            if (!stage["target_type"]) continue;
            if (stage["target_type"].as<std::string>() != "tf_frame") continue;
            if (!stage["target"]) continue;

            std::string target = TaskConfigInterpreter::resolveTemplateVar(
                stage["target"].as<std::string>(), m_stepParams);

            if (m_perception->m_transformedPoses.count(target)) {
                named_poses[target] = m_perception->m_transformedPoses[target];
            } else {
                RCLCPP_WARN(m_node->get_logger(),
                            "executeGenericMotionTask '%s': TF frame '%s' not in pose registry",
                            task_name.c_str(), target.c_str());
            }
        }
    }

    std::string name_copy = task_name;
    return doTask(name_copy, named_poses);
}

// ─── existing execute* implementations ────────────────────────────────────────

// ─── executeLocalizeBoard ─────────────────────────────────────────────────────
// Replicates the former BOARD_DETECTION→WAIT_FOR_RESPONSE→CHECK_TF state
// machine as a single retryable sequence step.  Returns false on each call
// until the board is fully localized (service responded + frame received + TF
// lookup succeeded), then returns true exactly once.
bool TaskOrchestrator::executeLocalizeBoard()
{
    // Phase 1: trigger the localize_board service; no-op if already waiting.
    if (!m_perception->m_service_map["localize_board"].srv_response.success) {
        m_perception->callTriggerService("localize_board");
        return false;
    }

    // Phase 2: wait for the frame_status topic to confirm TF frames are ready.
    if (!m_perception->m_frameStatus) {
        RCLCPP_INFO(m_node->get_logger(), "executeLocalizeBoard: waiting for frame_status");
        return false;
    }

    // Phase 3: reset service state then snapshot all board TF poses.
    m_perception->m_service_map["localize_board"].srv_response.success = false;
    m_perception->m_waiting_for_response = false;

    bool ok = m_perception->generateStaticTFPose();
    if (ok) {
        RCLCPP_INFO(m_node->get_logger(), "executeLocalizeBoard: board localized successfully");
    } else {
        RCLCPP_WARN(m_node->get_logger(), "executeLocalizeBoard: TF lookup failed, will retry");
    }
    return ok;
}

bool TaskOrchestrator::executeGoHome()
{
    // home_pose param selects the target named pose. Defaults to home_camera.
    if (!m_stepParams.count("home_pose"))
        m_stepParams["home_pose"] = "home_camera";

    RCLCPP_INFO(m_node->get_logger(), "executeGoHome: %s", m_stepParams.at("home_pose").c_str());
    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    std::string current_task = "go_home";
    return doTask(current_task, named_poses);
}

bool TaskOrchestrator::executeRetractStylus()
{
    RCLCPP_INFO(m_node->get_logger(), "executeRetractStylus");
    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    std::string current_task = "retract_stylus";
    return doTask(current_task, named_poses);
}

bool TaskOrchestrator::executeSingleButtonPress()
{
    // Template-driven single button press.  button_frame selects the target TF frame.
    std::string btn = m_stepParams.count("button_frame") ? m_stepParams.at("button_frame") : "blue_button";
    RCLCPP_INFO(m_node->get_logger(), "executeSingleButtonPress: %s", btn.c_str());

    std::map<std::string, geometry_msgs::msg::Pose> named_poses;
    named_poses[btn] = m_perception->m_transformedPoses[btn];

    std::string current_task = "press_button";
    return doTask(current_task, named_poses);
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
