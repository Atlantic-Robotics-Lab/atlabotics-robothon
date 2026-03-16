#include "gripper_controller.h"

GripperController::GripperController(rclcpp::Node* node, const YAML::Node& config)
    : m_node(node), m_config(config)
{
    m_gripperSrv = m_node->create_client<gripper_srv::srv::GripperService>("/gripper_service");
}

void GripperController::gripperService(bool& state)
{
    if (m_waiting_for_gripper_response) {
        RCLCPP_WARN(m_node->get_logger(), "Gripper Service call in progress, skipping new request");
        return;
    }
    if (!m_gripperSrv->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(m_node->get_logger(), "Gripper Service not available yet");
        return;
    }

    // Phase 3.7: read from robot.gripper (new schema), fall back to top-level gripper:
    YAML::Node gripper_cfg;
    if (m_config["robot"] && m_config["robot"]["gripper"])
        gripper_cfg = m_config["robot"]["gripper"];
    else if (m_config["gripper"])
        gripper_cfg = m_config["gripper"];
    else {
        RCLCPP_ERROR(m_node->get_logger(),
                     "No gripper config found (checked robot.gripper and gripper)");
        return;
    }

    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    if (state) // Open
    {
        request->position = gripper_cfg["open"]["position"].as<int>();
        request->speed    = gripper_cfg["open"]["speed"].as<int>();
        request->force    = gripper_cfg["open"]["force"].as<int>();
    }
    else // Close
    {
        request->position = gripper_cfg["close"]["position"].as<int>();
        request->speed    = gripper_cfg["close"]["speed"].as<int>();
        request->force    = gripper_cfg["close"]["force"].as<int>();
    }

    m_waiting_for_gripper_response = true;

    auto future = m_gripperSrv->async_send_request(
        request,
        [this](rclcpp::Client<gripper_srv::srv::GripperService>::SharedFuture future) {
            auto response = future.get();
            if (response->response == "Done") {
                RCLCPP_INFO(m_node->get_logger(), "Service succeeded: %s", response->response.c_str());
            } else {
                RCLCPP_ERROR(m_node->get_logger(), "Service failed: %s", response->response.c_str());
            }
            m_waiting_for_gripper_response = false;
        }
    );
}
