#include "task_config_interpreter.h"

TaskConfigInterpreter::TaskConfigInterpreter(rclcpp::Node* node, const YAML::Node& config)
    : m_node(node), m_config(config)
{
}

bool TaskConfigInterpreter::doTask(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypoints)
{
    RCLCPP_INFO(m_node->get_logger(), "Create task");
    task_ = createTask(current_task, waypoints);

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

mtc::Task TaskConfigInterpreter::createTask(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypoints)
{
    mtc::Task task;
    task.stages()->setName(current_task);
    task.loadRobotModel(m_node->shared_from_this());

    std::string hand_frame_config = "ee_gripper";

    if (m_config["tasks"][current_task]["hand_frame"])
    {
        hand_frame_config = m_config["tasks"][current_task]["hand_frame"].as<std::string>();
        RCLCPP_INFO(m_node->get_logger(), "hand_frame_config: %s", hand_frame_config.c_str());
    }

    const auto& arm_group_name = "ur_manipulator";
    const auto& hand_frame = hand_frame_config;

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("ik_frame", hand_frame);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(m_node->shared_from_this());
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

    std::map<std::string, mtc::solvers::PlannerInterfacePtr> planners;
    planners["cartesian"] = cartesian_planner;
    planners["interpolation"] = interpolation_planner;
    planners["sampling_planner"] = sampling_planner;

    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(.01);

    interpolation_planner->setMaxVelocityScalingFactor(0.2);
    interpolation_planner->setMaxAccelerationScalingFactor(0.2);

    addStagesFromYaml(task, m_config["tasks"][current_task], waypoints, planners, arm_group_name, hand_frame);
    return task;
}

void TaskConfigInterpreter::addStagesFromYaml(mtc::Task& task, const YAML::Node& task_config, const std::map<std::string, geometry_msgs::msg::Pose>& named_poses, std::map<std::string, mtc::solvers::PlannerInterfacePtr>& planners, const std::string& group_name, const std::string& hand_frame)
{
    RCLCPP_INFO(m_node->get_logger(), "addStagesFromYaml");
    for (const auto& stage_node : task_config["stages"])
    {
        std::string type = stage_node["type"].as<std::string>();
        std::string name = stage_node["name"].as<std::string>();
        std::string planner_name = stage_node["planner"].as<std::string>();
        double min_distance = 0.1;
        double max_distance = 0.1;

        std::string hand_frame_name = "";

        auto getPlanner = [&](const std::string& name) -> mtc::solvers::PlannerInterfacePtr {
            return planners.at(name);
        };

        auto planner = getPlanner(planner_name);
        if (stage_node["hand_frame"]) { hand_frame_name = stage_node["hand_frame"].as<std::string>(); }
        else hand_frame_name = hand_frame;

        if (stage_node["minmax_dist"])
        {
            min_distance = stage_node["minmax_dist"][0].as<double>();
            max_distance = stage_node["minmax_dist"][1].as<double>();
        }

        if (stage_node["vel_acc"])
        {
            planner->setMaxVelocityScalingFactor(stage_node["vel_acc"][0].as<double>());
            planner->setMaxAccelerationScalingFactor(stage_node["vel_acc"][1].as<double>());
        }

        RCLCPP_INFO(m_node->get_logger(), "TASK TYPE %s", type.c_str());

        if (type == "move_to")
        {
            std::string target_name = stage_node["target"].as<std::string>();
            auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);

            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setGroup(group_name);
            stage->setIKFrame(hand_frame);

            RCLCPP_INFO(m_node->get_logger(), "move_to target: %s", target_name.c_str());
            if (target_name != "home_camera_vertical" && target_name != "home_camera" && target_name != "home_camera_touch" && target_name != "home_camera_magnet" && target_name != "stylus_calibration")
            {
                auto pose = named_poses.at(target_name);
                double off_x = 0.0, off_y = 0.0, off_z = 0.0;
                if (stage_node["offset"]) {
                    off_x = stage_node["offset"][0].as<double>();
                    off_y = stage_node["offset"][1].as<double>();
                    off_z = stage_node["offset"][2].as<double>();
                }
                geometry_msgs::msg::PoseStamped offset_pose;
                offset_pose.header.frame_id = "base_link";
                offset_pose.pose = pose;
                offset_pose.pose.position.x += off_x;
                offset_pose.pose.position.y += off_y;
                offset_pose.pose.position.z += off_z;
                stage->setGoal(offset_pose);
            }
            else if (target_name == "home_camera") // Requires string parse from srdf
            {
                stage->setGoal("home_camera");
            }
            else if (target_name == "home_camera_touch")
            {
                stage->setGoal("home_camera_touch");
            }
            else if (target_name == "home_camera_magnet")
            {
                stage->setGoal("home_camera_magnet");
            }
            else if (target_name == "stylus_calibration")
            {
                stage->setGoal("stylus_calibration");
            }
            else
            {
                stage->setGoal("home_camera_vertical");
            }
            task.add(std::move(stage));
        }
        else if (type == "move_relative")
        {
            auto dir_vals = stage_node["direction"];
            std::string frame_id = "world";
            if (stage_node["frame"])
            {
                RCLCPP_INFO(m_node->get_logger(), "Frame defined: %s", stage_node["frame"].as<std::string>().c_str());
                frame_id = stage_node["frame"].as<std::string>();
            }
            geometry_msgs::msg::Vector3 dir;
            dir.x = dir_vals[0].as<double>();
            dir.y = dir_vals[1].as<double>();
            dir.z = dir_vals[2].as<double>();

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
        else
        {
            RCLCPP_INFO(m_node->get_logger(), "Creating serial ");
            auto path = std::make_unique<mtc::SerialContainer>("follow path");
            task.properties().exposeTo(path->properties(), { "group", "ik_frame" });
            path->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });

            std::string target_name = stage_node["target"].as<std::string>();
            int index = 0;
            for (auto& [path_point_name, path_point] : named_poses)
            {
                if (path_point_name == "screen" || path_point_name == "screen_align_pose")
                    continue;
                auto stage = std::make_unique<mtc::stages::MoveTo>(path_point_name, planner);
                RCLCPP_INFO(m_node->get_logger(), "target %s path_point_name %s", target_name.c_str(), path_point_name.c_str());
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
