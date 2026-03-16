#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "gripper_srv/srv/gripper_service.hpp"

class GripperController
{
public:
    explicit GripperController(rclcpp::Node* node, const YAML::Node& config);
    void gripperService(bool& state);

private:
    rclcpp::Node* m_node;
    YAML::Node m_config;
    rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr m_gripperSrv;
    bool m_waiting_for_gripper_response{false};
};
