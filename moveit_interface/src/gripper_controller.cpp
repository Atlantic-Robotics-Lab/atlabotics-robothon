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

    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    if (state) // Open
    {
        request->position = m_config["gripper"]["open"]["position"].as<int>();
        request->speed    = m_config["gripper"]["open"]["speed"].as<int>();
        request->force    = m_config["gripper"]["open"]["force"].as<int>();
    }
    else // Close
    {
        request->position = m_config["gripper"]["close"]["position"].as<int>();
        request->speed    = m_config["gripper"]["close"]["speed"].as<int>();
        request->force    = m_config["gripper"]["close"]["force"].as<int>();
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
