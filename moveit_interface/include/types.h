#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <string>
#include <vector>

enum class InterfaceState
{
    IDLE,
    BOARD_DETECTION,
    WAIT_FOR_RESPONSE,
    CHECK_TF,
    EXECUTE,
    DONE
};

enum TaskType
{
    NONE,
    SPEED_PRESS,
    PRESS_BUTTONS,
    GRAB_STYLUS_MAGNET,
    GRAB_STYLUS_TOUCH,
    SCREEN_SHAPE,
    SCREEN_TEXT,
    MAZE,
    BYOD,
    WAIT_FOR_SRV_RESPONSE,
    LOCALIZE_BOARD,
    DROP_STYLUS,
    END
};

enum ScreenTextTask
{
    MOVE=0,
    TAP=-1,
    COUNT=2
};

struct ServiceInfo {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    TaskType task_type;
    std::string name;
    std_srvs::srv::Trigger::Response srv_response;
};

struct ParsedTask {
    int id;
    std::vector<std::string> task_names;
};
