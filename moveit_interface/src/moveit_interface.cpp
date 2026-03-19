#include "moveit_interface.h"

MoveitInterface::MoveitInterface(const rclcpp::NodeOptions& options)
    : Node("moveit_interface", options)
{
    RCLCPP_INFO(this->get_logger(), "MoveitInterface node created");

    std::string pkg_share = ament_index_cpp::get_package_share_directory("moveit_interface");
    std::filesystem::path config_file = std::filesystem::path(pkg_share) / "config" / "move_group_params.yaml";
    m_yamlPath = config_file.string();
    m_config = YAML::LoadFile(m_yamlPath);

    // Construct sub-components after the node base is ready
    m_gripper     = std::make_unique<GripperController>(this, m_config);
    m_perception  = std::make_unique<PerceptionBridge>(this, m_config);
    m_interpreter = std::make_unique<TaskConfigInterpreter>(this, m_config);
    m_orchestrator = std::make_unique<TaskOrchestrator>(this, m_perception.get(), m_gripper.get(), m_interpreter.get());

    m_triggerTask = this->create_service<std_srvs::srv::Trigger>(
        "trigger_task",
        std::bind(&MoveitInterface::triggerTaskCallback, this, std::placeholders::_1, std::placeholders::_2));

    m_state    = InterfaceState::IDLE;
    m_taskType = TaskType::NONE;

    setupPlanningScene();
}

MoveitInterface::~MoveitInterface()
{
}

void MoveitInterface::triggerTaskCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_completed = false;
    if (!m_config["custom_task"].as<bool>())
    {
        m_state = InterfaceState::IDLE;
        m_orchestrator->reset();  // reset task sequence index so re-trigger starts from step 1
    }
    response->success = true;
    response->message = "Task state trigger set";
}

void MoveitInterface::setParams()
{
    auto mg = m_config["move_group"];
    if (!mg) {
        RCLCPP_ERROR(this->get_logger(), "Move group config missing in YAML!");
        return;
    }
    std::string planner_id    = m_config["move_group"]["planner_id"].as<std::string>();
    double planning_time      = m_config["move_group"]["planning_time"].as<double>();
    int num_attempts          = m_config["move_group"]["num_planning_attempts"].as<int>();
    m_maxVel = m_config["move_group"]["max_velocity_scaling_factor"].as<double>();
    m_maxAcc = m_config["move_group"]["max_acceleration_scaling_factor"].as<double>();
    std::string ee_link       = m_config["move_group"]["end_effector_link"].as<std::string>();
    std::string pose_ref      = m_config["move_group"]["pose_reference_frame"].as<std::string>();

    RCLCPP_INFO(this->get_logger(),
        "Loaded Move Group Config: planner=%s time=%.1f attempts=%d vel=%.2f acc=%.2f ee=%s ref=%s",
        planner_id.c_str(), planning_time, num_attempts, m_maxVel, m_maxAcc, ee_link.c_str(), pose_ref.c_str());

    if (!m_movegroupInterface) {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized!");
        return;
    }
    m_movegroupInterface->setPlannerId(planner_id);
    m_movegroupInterface->setPlanningTime(planning_time);
    m_movegroupInterface->setNumPlanningAttempts(num_attempts);
    m_movegroupInterface->setMaxVelocityScalingFactor(m_maxVel);
    m_movegroupInterface->setMaxAccelerationScalingFactor(m_maxAcc);
    m_movegroupInterface->setEndEffectorLink(ee_link);
    m_movegroupInterface->setPoseReferenceFrame(pose_ref);
}

void MoveitInterface::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object2;
    object2.id = "base_object";
    object2.header.frame_id = "world";
    object2.primitives.resize(1);
    object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object2.primitives[0].dimensions = { 2.0, 2.0, 0.05 };

    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.0;
    pose2.position.y = 0.0;
    pose2.position.z = -0.05;
    pose2.orientation.w = 1.0;
    object2.pose = pose2;

    scene_.applyCollisionObject(object2);
}

void MoveitInterface::run()
{
    // Initialize MoveGroupInterface once at startup so motion tasks are available immediately.
    m_movegroupInterface = std::make_shared<MoveGroupInterface>(this->shared_from_this(), "ur_manipulator");
    setParams();

    if (m_config["custom_task"].as<bool>())
    {
        m_nextTaskType = TaskType::BYOD;
        m_state = InterfaceState::EXECUTE;
    }

    m_completed = true;
    while (rclcpp::ok())
    {
        if (!m_completed)
        {
            switch (m_state)
            {
                case InterfaceState::IDLE:
                    RCLCPP_INFO(this->get_logger(), "State: IDLE -> EXECUTE");
                    m_state = InterfaceState::EXECUTE;
                    break;

                case InterfaceState::EXECUTE:
                {
                    m_orchestrator->executeTasks(m_nextTaskType, m_state);
                    break;
                }

                case InterfaceState::DONE:
                {
                    RCLCPP_INFO(this->get_logger(), "All tasks completed. Staying in DONE state.");
                    m_completed = true;
                    break;
                }
            }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}
