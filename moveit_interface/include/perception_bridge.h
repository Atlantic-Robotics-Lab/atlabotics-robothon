#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <map>
#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "types.h"

class PerceptionBridge
{
public:
    explicit PerceptionBridge(rclcpp::Node* node, const YAML::Node& config);

    // TF / pose lookup
    geometry_msgs::msg::Pose lookupPoseTransformStamped(std::string target_frame_id, std::string source_frame_id);
    void isTransformAvailable(geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, const std::string& target_frame, const std::string& source_frame, double timeout_sec);
    void getTf(geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, std::string target_frame, std::string source_frame);

    // Pose population
    bool generateStaticTFPose();
    void loadCSVToPoses(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& dummyPath);

    // Service calls
    void callTriggerService(const std::string& key);

    // Text command parsing
    void parseTaskCommand(std::string& taskCommand, ParsedTask& parsedTask);

    // Reset perception state (called by TaskOrchestrator::reset)
    void resetState();

    // Direct buffer access needed by execute* methods that call lookupTransform directly
    tf2_ros::Buffer& getTfBuffer() { return m_tfBuffer; }

    // Public state — read by TaskOrchestrator and MoveitInterface::run()
    std::unordered_map<std::string, ServiceInfo> m_service_map;
    bool m_frameStatus{false};
    bool m_waiting_for_response{false};
    bool m_tfFound{false};

    std::string m_labelData{""};
    std::string m_buttonStatus{""};
    std::string m_screenCommand{""};

    geometry_msgs::msg::PoseArray m_shapePoses;
    std::vector<geometry_msgs::msg::PoseArray> m_detectionPoses;
    std::vector<geometry_msgs::msg::Pose> m_mazePath;
    std::map<std::string, geometry_msgs::msg::Pose> m_transformedPoses;

private:
    rclcpp::Node* m_node;
    YAML::Node m_config;
    TaskType m_taskType{TaskType::NONE};

    tf2_ros::Buffer m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_frameSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_labelSub, m_buttonSub, m_textSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_pointsSub;

    // Named pose intermediates used by generateStaticTFPose
    geometry_msgs::msg::Pose m_redButtonPose, m_blueButtonPose, m_stylusPose, m_mazePose;
    geometry_msgs::msg::Pose m_screenPose, m_screenAlignPose;
    geometry_msgs::msg::Pose m_screenA, m_screenB, m_screenBackground;
    geometry_msgs::msg::Pose m_screenUp, m_screenDown, m_screenLeft, m_screenRight;
    geometry_msgs::msg::Pose m_preRedButtonPose, m_preBlueButtonPose, m_preStylusPose, m_preMazePose, m_preScreenPose;

    // Topic callbacks
    void frame_status_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void button_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void screenTextCallback(const std_msgs::msg::String::SharedPtr msg);
    void labelCallback(const std_msgs::msg::String::SharedPtr msg);
    void pointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
};
