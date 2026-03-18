#include "task_config_interpreter.h"

TaskConfigInterpreter::TaskConfigInterpreter(rclcpp::Node* node, const YAML::Node& config)
    : m_node(node), m_config(config)
{
}

// ─── Template variable resolution (Phase 4.1) ─────────────────────────────────
// Replaces all {{key}} placeholders in s with the value from params[key].
std::string TaskConfigInterpreter::resolveTemplateVar(
    const std::string& s,
    const std::map<std::string, std::string>& params)
{
    if (params.empty() || s.find("{{") == std::string::npos)
        return s;

    std::string result = s;
    for (const auto& [key, val] : params) {
        const std::string placeholder = "{{" + key + "}}";
        size_t pos = 0;
        while ((pos = result.find(placeholder, pos)) != std::string::npos)
            result.replace(pos, placeholder.size(), val);
    }
    return result;
}

// ─── doTask ───────────────────────────────────────────────────────────────────
bool TaskConfigInterpreter::doTask(
    std::string& current_task,
    std::map<std::string, geometry_msgs::msg::Pose>& waypoints,
    const std::map<std::string, std::string>& params)
{
    RCLCPP_INFO(m_node->get_logger(), "Create task");
    task_ = createTask(current_task, waypoints, params);

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), e);
        return false;
    }

    if (!task_.plan(15))
    {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Task planning failed");
        return false;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    if (m_config["execute"].as<bool>(true)) {
        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(m_node->get_logger(), "Task execution failed");
            return false;
        }
    }

    return true;
}

// ─── createTask ───────────────────────────────────────────────────────────────
mtc::Task TaskConfigInterpreter::createTask(
    std::string& current_task,
    std::map<std::string, geometry_msgs::msg::Pose>& waypoints,
    const std::map<std::string, std::string>& params)
{
    mtc::Task task;
    task.stages()->setName(current_task);
    task.loadRobotModel(m_node->shared_from_this());

    // ── Arm group (Phase 3.7: read from robot.arm_group, hardcoded fallback) ──
    std::string arm_group_name = "ur_manipulator";
    if (m_config["robot"] && m_config["robot"]["arm_group"])
        arm_group_name = m_config["robot"]["arm_group"].as<std::string>();

    // ── End-effector / hand frame (Phase 3.3) ────────────────────────────────
    // Priority: end_effector (new schema) → hand_frame (legacy) → default "ee_gripper"
    std::string hand_frame_config = "ee_gripper";
    const YAML::Node& task_node = m_config["tasks"][current_task];
    if (task_node["end_effector"]) {
        std::string ee_name = task_node["end_effector"].as<std::string>();
        if (m_config["robot"]["end_effectors"][ee_name]["frame_id"]) {
            hand_frame_config = m_config["robot"]["end_effectors"][ee_name]["frame_id"].as<std::string>();
            RCLCPP_INFO(m_node->get_logger(), "end_effector: %s → frame_id: %s",
                        ee_name.c_str(), hand_frame_config.c_str());
        } else {
            RCLCPP_WARN(m_node->get_logger(),
                        "end_effector '%s' not found in robot.end_effectors, using default",
                        ee_name.c_str());
        }
    } else if (task_node["hand_frame"]) {
        hand_frame_config = task_node["hand_frame"].as<std::string>();
        RCLCPP_INFO(m_node->get_logger(), "hand_frame_config (legacy): %s", hand_frame_config.c_str());
    }

    const auto& hand_frame = hand_frame_config;

    task.setProperty("group", arm_group_name);
    task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // ── Planner defaults (Phase 3.7: read from robot.planners) ──────────────
    double cartesian_step = 0.01;
    double cartesian_vel  = 0.2, cartesian_acc  = 0.2;
    double interp_vel     = 0.2, interp_acc     = 0.2;

    if (m_config["robot"] && m_config["robot"]["planners"]) {
        const auto& planners_cfg = m_config["robot"]["planners"];
        if (planners_cfg["cartesian"]) {
            const auto& cp = planners_cfg["cartesian"];
            if (cp["step_size"])         cartesian_step = cp["step_size"].as<double>();
            if (cp["default_vel_acc"]) {
                cartesian_vel = cp["default_vel_acc"][0].as<double>();
                cartesian_acc = cp["default_vel_acc"][1].as<double>();
            }
        }
        if (planners_cfg["interpolation"] && planners_cfg["interpolation"]["default_vel_acc"]) {
            const auto& ip = planners_cfg["interpolation"]["default_vel_acc"];
            interp_vel = ip[0].as<double>();
            interp_acc = ip[1].as<double>();
        }
    }

    auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(m_node->shared_from_this());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner     = std::make_shared<mtc::solvers::CartesianPath>();

    cartesian_planner->setMaxVelocityScalingFactor(cartesian_vel);
    cartesian_planner->setMaxAccelerationScalingFactor(cartesian_acc);
    cartesian_planner->setStepSize(cartesian_step);

    interpolation_planner->setMaxVelocityScalingFactor(interp_vel);
    interpolation_planner->setMaxAccelerationScalingFactor(interp_acc);

    std::map<std::string, mtc::solvers::PlannerInterfacePtr> planners;
    planners["cartesian"]        = cartesian_planner;
    planners["interpolation"]    = interpolation_planner;
    planners["sampling_planner"] = sampling_planner;

    addStagesFromYaml(task, task_node, waypoints, planners, arm_group_name, hand_frame, params);
    return task;
}

// ─── addStagesFromYaml ────────────────────────────────────────────────────────
void TaskConfigInterpreter::addStagesFromYaml(
    mtc::Task& task,
    const YAML::Node& task_config,
    const std::map<std::string, geometry_msgs::msg::Pose>& named_poses,
    std::map<std::string, mtc::solvers::PlannerInterfacePtr>& planners,
    const std::string& group_name,
    const std::string& hand_frame,
    const std::map<std::string, std::string>& params)
{
    RCLCPP_INFO(m_node->get_logger(), "addStagesFromYaml");

    // ── Read per-planner defaults once so each stage gets a clean baseline ────
    // Without this, a stage that sets vel_acc mutates the shared planner object
    // and subsequent stages using the same planner inherit those values instead
    // of the YAML-configured defaults.
    double cart_vel_def = 0.2, cart_acc_def = 0.2;
    double interp_vel_def = 0.2, interp_acc_def = 0.2;
    if (m_config["robot"] && m_config["robot"]["planners"]) {
        const auto& pcfg = m_config["robot"]["planners"];
        if (pcfg["cartesian"] && pcfg["cartesian"]["default_vel_acc"]) {
            cart_vel_def = pcfg["cartesian"]["default_vel_acc"][0].as<double>();
            cart_acc_def = pcfg["cartesian"]["default_vel_acc"][1].as<double>();
        }
        if (pcfg["interpolation"] && pcfg["interpolation"]["default_vel_acc"]) {
            interp_vel_def = pcfg["interpolation"]["default_vel_acc"][0].as<double>();
            interp_acc_def = pcfg["interpolation"]["default_vel_acc"][1].as<double>();
        }
    }

    for (const auto& stage_node : task_config["stages"])
    {
        std::string type         = stage_node["type"].as<std::string>();
        std::string name         = stage_node["name"].as<std::string>();
        std::string planner_name = stage_node["planner"].as<std::string>();
        double min_distance = 0.1;
        double max_distance = 0.1;

        std::string hand_frame_name = hand_frame;
        if (stage_node["hand_frame"])
            hand_frame_name = stage_node["hand_frame"].as<std::string>();

        // Guard against unknown planner names before accessing the map
        if (!planners.count(planner_name)) {
            RCLCPP_ERROR(m_node->get_logger(),
                         "Unknown planner '%s' in stage '%s' — skipping",
                         planner_name.c_str(), name.c_str());
            continue;
        }
        auto planner = planners.at(planner_name);

        if (stage_node["minmax_dist"])
        {
            min_distance = stage_node["minmax_dist"][0].as<double>();
            max_distance = stage_node["minmax_dist"][1].as<double>();
        }

        // Reset to YAML defaults first, then apply stage-level override if present.
        // This ensures each stage starts from a known baseline regardless of what
        // the previous stage set on the same shared planner object.
        if (planner_name == "cartesian") {
            planner->setMaxVelocityScalingFactor(cart_vel_def);
            planner->setMaxAccelerationScalingFactor(cart_acc_def);
        } else if (planner_name == "interpolation") {
            planner->setMaxVelocityScalingFactor(interp_vel_def);
            planner->setMaxAccelerationScalingFactor(interp_acc_def);
        }

        if (stage_node["vel_acc"])
        {
            planner->setMaxVelocityScalingFactor(stage_node["vel_acc"][0].as<double>());
            planner->setMaxAccelerationScalingFactor(stage_node["vel_acc"][1].as<double>());
        }

        RCLCPP_INFO(m_node->get_logger(), "TASK TYPE %s", type.c_str());

        // ── move_to ──────────────────────────────────────────────────────────
        if (type == "move_to")
        {
            // Resolve template variables in target name (Phase 4.1)
            std::string target_name = resolveTemplateVar(
                stage_node["target"].as<std::string>(), params);

            // ── Determine target_type (Phase 3.1) ────────────────────────────
            // Explicit target_type in YAML takes priority.
            // Fallback infers from the hardcoded SRDF named-pose list for backward compat.
            std::string target_type = "";
            if (stage_node["target_type"])
                target_type = stage_node["target_type"].as<std::string>();

            if (target_type.empty()) {
                target_type = (target_name == "home_camera_vertical" ||
                               target_name == "home_camera"          ||
                               target_name == "home_camera_touch"    ||
                               target_name == "home_camera_magnet"   ||
                               target_name == "stylus_calibration")
                              ? "named_pose" : "tf_frame";
            }

            RCLCPP_INFO(m_node->get_logger(), "move_to target: %s (target_type: %s)",
                        target_name.c_str(), target_type.c_str());

            auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setGroup(group_name);
            stage->setIKFrame(hand_frame);

            if (target_type == "named_pose") {
                // SRDF joint state — set goal by name
                stage->setGoal(target_name);
                task.add(std::move(stage));
            } else {
                // tf_frame — look up pose in named_poses map
                auto it = named_poses.find(target_name);
                if (it == named_poses.end()) {
                    RCLCPP_ERROR(m_node->get_logger(),
                                 "Pose '%s' not in named_poses, skipping stage '%s'",
                                 target_name.c_str(), name.c_str());
                    // stage not added — skips this waypoint gracefully
                } else {
                    double off_x = 0.0, off_y = 0.0, off_z = 0.0;
                    if (stage_node["offset"]) {
                        off_x = stage_node["offset"][0].as<double>();
                        off_y = stage_node["offset"][1].as<double>();
                        off_z = stage_node["offset"][2].as<double>();
                    }

                    // constrain_orientation: true  → PoseStamped (position + orientation).
                    //   Use when the gripper must arrive in a specific orientation, e.g.
                    //   stylus pick where subsequent align stages move in the ee frame.
                    // constrain_orientation: false (default) → PointStamped (position only).
                    //   IK solver is free to choose orientation — good for button presses
                    //   where approach direction doesn't matter.
                    bool constrain_orientation = false;
                    if (stage_node["constrain_orientation"])
                        constrain_orientation = stage_node["constrain_orientation"].as<bool>();

                    if (constrain_orientation) {
                        geometry_msgs::msg::PoseStamped target_pose;
                        target_pose.header.frame_id = "base_link";
                        target_pose.pose             = it->second;
                        target_pose.pose.position.x += off_x;
                        target_pose.pose.position.y += off_y;
                        target_pose.pose.position.z += off_z;
                        stage->setGoal(target_pose);
                    } else {
                        geometry_msgs::msg::PointStamped target_point;
                        target_point.header.frame_id = "base_link";
                        target_point.point.x = it->second.position.x + off_x;
                        target_point.point.y = it->second.position.y + off_y;
                        target_point.point.z = it->second.position.z + off_z;
                        stage->setGoal(target_point);
                    }
                    task.add(std::move(stage));
                }
            }
        }
        // ── move_relative ────────────────────────────────────────────────────
        else if (type == "move_relative")
        {
            auto dir_vals = stage_node["direction"];

            // Phase 3.5: frame is now explicit in YAML; resolve template vars in it
            std::string frame_id = "world";
            if (stage_node["frame"])
                frame_id = resolveTemplateVar(stage_node["frame"].as<std::string>(), params);

            geometry_msgs::msg::Vector3 dir;
            dir.x = dir_vals[0].as<double>();
            dir.y = dir_vals[1].as<double>();
            dir.z = dir_vals[2].as<double>();

            RCLCPP_INFO(m_node->get_logger(), "Frame defined: %s", frame_id.c_str());

            auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setGroup(group_name);
            stage->setMinMaxDistance(min_distance, max_distance);
            stage->setIKFrame(hand_frame);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = frame_id;
            vec.vector = dir;
            stage->setDirection(vec);

            task.add(std::move(stage));
        }
        // ── move_to_path (serial container of sequential waypoints) ──────────
        else
        {
            RCLCPP_INFO(m_node->get_logger(), "Creating serial path container");
            auto path = std::make_unique<mtc::SerialContainer>("follow path");
            task.properties().exposeTo(path->properties(), { "group", "ik_frame" });
            path->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });

            std::string target_name = resolveTemplateVar(
                stage_node["target"].as<std::string>(), params);
            int index = 0;
            for (auto& [path_point_name, path_point] : named_poses)
            {
                if (path_point_name == "screen" || path_point_name == "screen_align_pose")
                    continue;
                auto stage = std::make_unique<mtc::stages::MoveTo>(path_point_name, planner);
                RCLCPP_INFO(m_node->get_logger(), "target %s path_point_name %s",
                            target_name.c_str(), path_point_name.c_str());
                stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                stage->setGroup(group_name);
                stage->setIKFrame(hand_frame);

                auto pose = named_poses.at(path_point_name);
                auto offset_vals = stage_node["offset"];
                geometry_msgs::msg::PointStamped offset_pose;
                offset_pose.header.frame_id = "base_link";
                offset_pose.point.x = pose.position.x + offset_vals[0].as<double>();
                offset_pose.point.y = pose.position.y + offset_vals[1].as<double>();
                offset_pose.point.z = pose.position.z + offset_vals[2].as<double>();
                stage->setGoal(offset_pose);
                path->insert(std::move(stage));
                index++;
            }
            task.add(std::move(path));
        }
    }
}
