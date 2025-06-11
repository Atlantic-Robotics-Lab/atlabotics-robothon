
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>  // <---- add this to the set of includes at the top
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <geometry_msgs/msg/pose_array.hpp>

// #include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <map>
//write to file
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/archive/binary_oarchive.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>  // C++17

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "std_srvs/srv/trigger.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>

#define BUTTON_SRV = "/taskboard/detect_button"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stage.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <filesystem>  // C++17
#include <fstream>
#include <Eigen/Geometry>
#include <sstream>

#include "gripper_srv/srv/gripper_service.hpp"

using moveit::planning_interface::MoveGroupInterface;
namespace mtc = moveit::task_constructor;


// class MoveitInterface: public rclcpp::Node, public std::enable_shared_from_this<MoveitInterface>
class MoveitInterface: public rclcpp::Node
{
    public:
        explicit MoveitInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~MoveitInterface();
        void printRobotTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory);
        void writeTrajectoryToPickle(const moveit_msgs::msg::RobotTrajectory &trajectory);
        bool createWaypointTrajectory(std::string&, std::map<std::string, geometry_msgs::msg::Pose>&);
        bool createWaypointTrajectory(std::vector<geometry_msgs::msg::Pose>&);
        void planTrajectory(bool &validTrajectory);
        void run();
        void frame_status_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void button_status_callback(const std_msgs::msg::String::SharedPtr msg);
        void screenTextCallback(const std_msgs::msg::String::SharedPtr msg);
        void labelCallback(const std_msgs::msg::String::SharedPtr msg);
        void pointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        void triggerTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        bool doTask(std::string&, std::map<std::string, geometry_msgs::msg::Pose>&);
        mtc::Task createTask(std::string&, std::map<std::string, geometry_msgs::msg::Pose>&);
        geometry_msgs::msg::Pose lookupPoseTransformStamped(std::string target_frame_id, std::string source_frame_id);
        void setupPlanningScene();
        
        std::shared_ptr<MoveGroupInterface> m_movegroupInterface;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_triggerTask;
        rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr m_gripperSrv;
        mtc::Task task_;

        moveit::planning_interface::PlanningSceneInterface scene_;


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

        std::unordered_map<std::string, ServiceInfo> m_service_map;
        ParsedTask m_parsedTask;

    private:
        void setParams();
        std::vector<geometry_msgs::msg::Pose> m_poseArray;
        moveit_msgs::msg::RobotTrajectory m_robotTrajectory;
        YAML::Node m_config;
        double m_maxVel = 0.0;
        double m_maxAcc = 0.0;
        int m_srvAttempts = 0;
        int m_screenTaskCounter = 0;
        int m_maxAttempt = 2;
        std::string m_yamlPath;
        std::string m_labelData = "";
        std::string m_buttonStatus ="";
        std::string m_screenCommand ="";

        bool m_tfFound{false};
        bool m_waiting_for_response{false};
        bool m_waiting_for_gripper_response{false};
        bool m_frameStatus{false};
        bool m_callShapeService{false};
        bool m_callTextService{false};
        bool m_completed{false};

        geometry_msgs::msg::PoseArray m_shapePoses;
        std::vector<geometry_msgs::msg::PoseArray> m_detectionPoses;

        tf2_ros::Buffer m_tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
        InterfaceState m_state{InterfaceState::IDLE};
        TaskType m_taskType{TaskType::NONE};
        TaskType m_nextTaskType{TaskType::NONE};
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_frameSub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_labelSub,m_buttonSub, m_textSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_pointsSub;
        geometry_msgs::msg::Pose m_redButtonPose, m_blueButtonPose, m_stylusPose, m_mazePose, m_screenPose, m_screenAlignPose;
        geometry_msgs::msg::Pose m_screenA, m_screenB, m_screenBackground, m_screenUp, m_screenDown, m_screenLeft, m_screenRIght;
        geometry_msgs::msg::Pose m_preRedButtonPose, m_preBlueButtonPose, m_preStylusPose, m_preMazePose, m_preScreenPose;
        std::map<std::string, geometry_msgs::msg::Pose> m_transformedPoses; 
        std::vector<geometry_msgs::msg::Pose> m_mazePath;
        
        void isTransformAvailable(geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, const std::string& target_frame, const std::string& source_frame, double timeout_sec);
        void gripperService(bool&);
        void getTf(geometry_msgs::msg::Pose& input_pose,geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, std::string target_frame,std::string source_frame);
        void loadCSVToPoses(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& dummyPath);
        void executeTasks(TaskType& nextTask);
        bool executeButtonPress();
        bool executeScreenMotion();
        bool executeScreenText();
        bool executeSpeedPress();
        bool executeMaze();
        bool executeDropStylus();
        bool executeGrabStylus(TaskType&);
        void callTriggerService(const std::string& key);
        bool generateStaticTFPose();

        void addStagesFromYaml(mtc::Task& task, const YAML::Node& task_config, const std::map<std::string, geometry_msgs::msg::Pose>& named_poses, std::map<std::string, mtc::solvers::PlannerInterfacePtr>& planners, const std::string& group_name, const std::string& ik_frame);
            
        void parseTaskCommand(std::string&,ParsedTask&);
        bool executeCustomTask();
        void reset();
};
