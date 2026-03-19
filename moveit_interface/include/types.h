#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <map>
#include <string>
#include <vector>

enum class InterfaceState
{
    IDLE,
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

// One entry in task_sequence YAML (Phase 3.6 / 4.1)
struct TaskSequenceEntry {
    std::string type;
    std::map<std::string, std::string> params;
};
