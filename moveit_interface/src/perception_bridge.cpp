#include "perception_bridge.h"

PerceptionBridge::PerceptionBridge(rclcpp::Node* node, const YAML::Node& config)
    : m_node(node)
    , m_config(config)
    , m_tfBuffer(node->get_clock())
    , m_tfListener(std::make_shared<tf2_ros::TransformListener>(m_tfBuffer, node, true))
{
    m_service_map["localize_board"] = {
        m_node->create_client<std_srvs::srv::Trigger>("/taskboard/board_localization"),
        TaskType::LOCALIZE_BOARD,
        "localize_board",
    };
    m_service_map["detect_button"] = {
        m_node->create_client<std_srvs::srv::Trigger>("/taskboard/detect_button"),
        TaskType::PRESS_BUTTONS,
        "detect_button"
    };
    m_service_map["detect_shape"] = {
        m_node->create_client<std_srvs::srv::Trigger>("/taskboard/detect_shape"),
        TaskType::SCREEN_SHAPE,
        "detect_shape"
    };
    m_service_map["detect_text"] = {
        m_node->create_client<std_srvs::srv::Trigger>("/trigger_pipeline"),
        TaskType::SCREEN_TEXT,
        "detect_text"
    };
    m_service_map["color_sort"] = {
        m_node->create_client<std_srvs::srv::Trigger>("/color_sort_service"),
        TaskType::BYOD,
        "color_sort"
    };

    m_frameSub = m_node->create_subscription<std_msgs::msg::Bool>("frame_status", 10,
        std::bind(&PerceptionBridge::frame_status_callback, this, std::placeholders::_1));

    m_buttonSub = m_node->create_subscription<std_msgs::msg::String>("button_status", 10,
        std::bind(&PerceptionBridge::button_status_callback, this, std::placeholders::_1));

    m_labelSub = m_node->create_subscription<std_msgs::msg::String>("detection_label", 10,
        std::bind(&PerceptionBridge::labelCallback, this, std::placeholders::_1));

    m_textSub = m_node->create_subscription<std_msgs::msg::String>("structured_command", 10,
        std::bind(&PerceptionBridge::screenTextCallback, this, std::placeholders::_1));

    m_pointsSub = m_node->create_subscription<geometry_msgs::msg::PoseArray>("detection_point", 10,
        std::bind(&PerceptionBridge::pointsCallback, this, std::placeholders::_1));
}

void PerceptionBridge::frame_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        RCLCPP_INFO(m_node->get_logger(), "Frames are ready (status: TRUE).");
        m_frameStatus = msg->data;
    }
    else
    {
        RCLCPP_WARN(m_node->get_logger(), "Frames not ready (status: FALSE).");
    }
}

void PerceptionBridge::button_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    m_buttonStatus = msg->data.c_str();
}

void PerceptionBridge::screenTextCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(m_node->get_logger(), "Text Detection : %s", msg->data.c_str());
    m_screenCommand = msg->data;
}

void PerceptionBridge::labelCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(m_node->get_logger(), "Detection label: %s", msg->data.c_str());
    m_labelData = msg->data;
}

void PerceptionBridge::pointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    RCLCPP_INFO(m_node->get_logger(), "Received %zu detection points.", msg->poses.size());
    if (msg->poses.empty())
    {
        m_shapePoses.poses.clear();
        m_shapePoses.poses.resize(0);
    }
    else
    {
        m_shapePoses = *msg;
        m_detectionPoses.push_back(*msg);
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto& pose = msg->poses[i];
            RCLCPP_INFO(m_node->get_logger(), "Point[%zu]: [%.2f, %.2f, %.2f]", i, pose.position.x, pose.position.y, pose.position.z);
        }
    }
}

void PerceptionBridge::isTransformAvailable(geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, const std::string& target_frame, const std::string& source_frame, double timeout_sec)
{
    m_tfFound = false;
    rclcpp::Time time_now = m_node->get_clock()->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
    try
    {
        tfstamped = m_tfBuffer.lookupTransform(target_frame, source_frame, time_now, rclcpp::Duration::from_seconds(0.5));
        tf2::doTransform(input_pose, output_pose, tfstamped);
        m_tfFound = true;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(m_node->get_logger(), "TF lookup failed: %s", ex.what());
        m_tfFound = false;
    }
}

void PerceptionBridge::getTf(geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, std::string target_frame, std::string source_frame)
{
    isTransformAvailable(input_pose, output_pose, tfstamped, target_frame, source_frame, 0.5);
    if (m_tfFound)
    {
        m_tfFound = false;
    }
    else
    {
        RCLCPP_ERROR(m_node->get_logger(), "Path Transform not found.");
    }
}

geometry_msgs::msg::Pose PerceptionBridge::lookupPoseTransformStamped(std::string target_frame_id, std::string source_frame_id)
{
    geometry_msgs::msg::Pose tfPose;
    try
    {
        geometry_msgs::msg::TransformStamped tfstamped = m_tfBuffer.lookupTransform(target_frame_id, source_frame_id, m_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
        tfPose.position.x = tfstamped.transform.translation.x;
        tfPose.position.y = tfstamped.transform.translation.y;
        tfPose.position.z = tfstamped.transform.translation.z;
        tfPose.orientation.x = tfstamped.transform.rotation.x;
        tfPose.orientation.y = tfstamped.transform.rotation.y;
        tfPose.orientation.z = tfstamped.transform.rotation.z;
        tfPose.orientation.w = tfstamped.transform.rotation.w;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN(rclcpp::get_logger("PoseLookup"), "TF lookup failed from %s to %s: %s", source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
    }
    return tfPose;
}

void PerceptionBridge::loadCSVToPoses(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& dummyPath)
{
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> data;

        // Parse comma-separated values into the `data` vector
        while (std::getline(ss, value, ',')) {
            value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
            if (value.empty()) continue;
            data.push_back(std::stod(value));
        }

        if (data.size() == 3) {
            geometry_msgs::msg::Pose test_pose;
            test_pose.position.x = data[0];
            test_pose.position.y = data[1];
            test_pose.position.z = data[2];
            test_pose.orientation.w = 1.0;
            dummyPath.push_back(test_pose);
        } else {
            RCLCPP_WARN(m_node->get_logger(), "Skipping malformed line: %s", line.c_str());
        }
    }
}

bool PerceptionBridge::generateStaticTFPose()
{
    std::string target_frame = "base_link";
    std::string source_frame = "";

    try
    {
        source_frame = "blue_button";
        m_blueButtonPose = lookupPoseTransformStamped(target_frame, source_frame);
        m_transformedPoses[source_frame] = m_blueButtonPose;

        source_frame = "red_button";
        m_redButtonPose = lookupPoseTransformStamped(target_frame, source_frame);
        m_transformedPoses[source_frame] = m_redButtonPose;

        source_frame = "stylus";
        m_stylusPose = lookupPoseTransformStamped(target_frame, source_frame);
        m_transformedPoses[source_frame] = m_stylusPose;

        source_frame = "maze";
        m_mazePose = lookupPoseTransformStamped(target_frame, source_frame);
        m_transformedPoses[source_frame] = m_mazePose;

        source_frame = "align_frame";
        m_screenAlignPose = lookupPoseTransformStamped(target_frame, source_frame);
        m_transformedPoses[source_frame] = m_screenAlignPose;

        source_frame = "screen";
        m_screenPose = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenPose.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenPose;

        source_frame = "A";
        m_screenA = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenA.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenA;

        source_frame = "B";
        m_screenB = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenB.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenB;

        source_frame = "Background";
        m_screenBackground = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenBackground.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenBackground;

        source_frame = "sq_up";
        m_screenUp = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenUp.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenUp;

        source_frame = "sq_down";
        m_screenDown = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenDown.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenDown;

        source_frame = "sq_left";
        m_screenLeft = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenLeft.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenLeft;

        source_frame = "sq_right";
        m_screenRight = lookupPoseTransformStamped(target_frame, source_frame);
        m_screenRight.orientation = m_screenAlignPose.orientation;
        m_transformedPoses[source_frame] = m_screenRight;

        loadCSVToPoses("/home/atu-2/robothon/src/moveit_interface/config/maze_path.csv", m_mazePath);

        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_INFO(m_node->get_logger(), "Could not transform 'base_link' to 'screen': %s", ex.what());
        return false;
    }
}

void PerceptionBridge::callTriggerService(const std::string& key)
{
    if (m_waiting_for_response) {
        RCLCPP_WARN(m_node->get_logger(), "[%s] Waiting for previous service response.", key.c_str());
        return;
    }

    if (m_service_map.find(key) == m_service_map.end()) {
        RCLCPP_ERROR(m_node->get_logger(), "Service key [%s] not found.", key.c_str());
        return;
    }

    auto& info = m_service_map[key];

    if (!info.client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(m_node->get_logger(), "[%s] Service not available", key.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    m_waiting_for_response = true;
    m_taskType = info.task_type;

    info.client->async_send_request(request,
    [this, key, &info](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
            auto response = future.get();
            info.srv_response.success = response->success;
            info.srv_response.message = response->message;

            if (response->success) {
                RCLCPP_INFO(m_node->get_logger(), "[%s] Service succeeded: %s", key.c_str(), response->message.c_str());
            } else {
                RCLCPP_ERROR(m_node->get_logger(), "[%s] Service failed: %s", key.c_str(), response->message.c_str());
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(m_node->get_logger(), "[%s] Exception: %s", key.c_str(), e.what());
        }
        m_waiting_for_response = false;
    });
}

void PerceptionBridge::parseTaskCommand(std::string& taskCommand, ParsedTask& parsedTask)
{
    std::vector<std::string> taskTokens;
    std::string taskCmd = taskCommand;
    std::stringstream ss(taskCmd);
    while (std::getline(ss, taskCmd, ',')) {
        taskCmd.erase(0, taskCmd.find_first_not_of(" \t"));
        taskCmd.erase(taskCmd.find_last_not_of(" \t") + 1);
        taskTokens.push_back(taskCmd);
    }

    if (!taskTokens.empty())
    {
        parsedTask.id = std::stoi(taskTokens[0]);
        int repeat_count = (parsedTask.id == -1) ? 2 : std::max(1, parsedTask.id);

        for (size_t i = 1; i < taskTokens.size(); ++i) {
            const std::string& task_name = taskTokens[i];
            for (int j = 0; j < repeat_count; ++j) {
                RCLCPP_INFO(m_node->get_logger(), "Adding task: %s [repeat %d]", task_name.c_str(), j + 1);
                parsedTask.task_names.push_back(task_name);
            }
        }
    }
}

void PerceptionBridge::resetState()
{
    m_frameStatus = false;
    m_mazePath.clear();
    m_mazePath.resize(0);
    m_detectionPoses.clear();
    m_detectionPoses.resize(0);
}
