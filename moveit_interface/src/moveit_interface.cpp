#include <moveit_interface.h>

MoveitInterface::MoveitInterface(const rclcpp::NodeOptions & options): Node("moveit_interface",options), m_tfBuffer(this->get_clock()), m_tfListener(std::make_shared<tf2_ros::TransformListener>(m_tfBuffer, this, true))
{
	RCLCPP_INFO(this->get_logger(), "MoveitInterface node created");
	std::string pkg_share = ament_index_cpp::get_package_share_directory("moveit_interface");
	std::filesystem::path config_file = std::filesystem::path(pkg_share) / "config" / "move_group_params.yaml";
	m_yamlPath = config_file.string();

	m_config = YAML::LoadFile(m_yamlPath);

	m_service_map["localize_board"] = {
		this->create_client<std_srvs::srv::Trigger>("/taskboard/board_localization"),
		TaskType::LOCALIZE_BOARD,
		"localize_board",
	};
	m_service_map["detect_button"] = {
		this->create_client<std_srvs::srv::Trigger>("/taskboard/detect_button"),
		TaskType::PRESS_BUTTONS,
		"detect_button"
	};

	m_service_map["detect_shape"] = {
		this->create_client<std_srvs::srv::Trigger>("/taskboard/detect_shape"),
		TaskType::SCREEN_SHAPE,
		"detect_shape"
	};
	m_service_map["detect_text"] = {
		this->create_client<std_srvs::srv::Trigger>("/trigger_pipeline"),
		TaskType::SCREEN_TEXT,
		"detect_text"
	};

	m_service_map["color_sort"] = {
		this->create_client<std_srvs::srv::Trigger>("/color_sort_service"),
		TaskType::BYOD,
		"color_sort"
	};

	m_gripperSrv = this->create_client<gripper_srv::srv::GripperService>("/gripper_service");

	m_triggerTask = this->create_service<std_srvs::srv::Trigger>("trigger_task",std::bind(&MoveitInterface::triggerTaskCallback, this, std::placeholders::_1, std::placeholders::_2));

	m_frameSub = this->create_subscription<std_msgs::msg::Bool>("frame_status", 10,
			std::bind(&MoveitInterface::frame_status_callback, this, std::placeholders::_1)
			);

	m_buttonSub = this->create_subscription<std_msgs::msg::String>("button_status", 10,
			std::bind(&MoveitInterface::button_status_callback, this, std::placeholders::_1)
			);

	m_labelSub = this->create_subscription<std_msgs::msg::String>("detection_label",10,
			std::bind(&MoveitInterface::labelCallback, this, std::placeholders::_1)
			);

	m_textSub = this->create_subscription<std_msgs::msg::String>("structured_command",10,
			std::bind(&MoveitInterface::screenTextCallback, this, std::placeholders::_1)
			);

	m_pointsSub = this->create_subscription<geometry_msgs::msg::PoseArray>("detection_point",10,
			std::bind(&MoveitInterface::pointsCallback, this, std::placeholders::_1)
			);

	m_state = InterfaceState::IDLE;
	m_taskType = TaskType::NONE;

	setupPlanningScene();

}

MoveitInterface::~MoveitInterface()
{

}

void MoveitInterface::triggerTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	m_completed = false;
	if(!m_config["custom_task"].as<bool>())
	{
		m_state = InterfaceState::IDLE;
	}	
	response->success = true;
	response->message = "Task tate trigger set";
}


void MoveitInterface::labelCallback(const std_msgs::msg::String::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Detection label: %s", msg->data.c_str());
	m_labelData = msg->data;
}

void MoveitInterface::screenTextCallback(const std_msgs::msg::String::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Text Detection : %s", msg->data.c_str());
	m_screenCommand = msg->data;
}

void MoveitInterface::pointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Received %zu detection points.", msg->poses.size());
	if(msg->poses.empty())
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
			RCLCPP_INFO(this->get_logger(), "Point[%zu]: [%.2f, %.2f, %.2f]", i, pose.position.x, pose.position.y, pose.position.z);
		}
	}
}

void MoveitInterface::frame_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	if (msg->data)
	{
		RCLCPP_INFO(this->get_logger(), "Frames are ready (status: TRUE).");
		m_frameStatus = msg->data;
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "Frames not ready (status: FALSE).");
	}

}

void MoveitInterface::button_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
	m_buttonStatus = msg->data.c_str();
}

void MoveitInterface::printRobotTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
	RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "Trajectory points:");
	//   for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
	//       const auto &point = trajectory.joint_trajectory.points[i];
	//       RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "Point %zu:", i + 1);
	//       RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "Positions:");
	//       for (size_t j = 0; j < point.positions.size(); ++j) {
	//           RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "  Joint %zu: %f", j + 1, point.positions[j]);
	//       }
	//   }

	//   for (size_t i = 0; i < trajectory.multi_dof_joint_trajectory.points.size(); ++i) {
	//     const auto &point = trajectory.multi_dof_joint_trajectory.points[i];
	//     RCLCPP_INFO(rclcpp::get_logger("multi_dof_joint_trajectory"), "Point %zu:", i + 1);
	//     RCLCPP_INFO(rclcpp::get_logger("multi_dof_joint_trajectory"), "Positions:");
	//     for (size_t j = 0; j < point.transforms.size(); ++j) {
	//         RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "  TF trans: %f %f %f ", point.transforms[j].translation.x, point.transforms[j].translation.y, point.transforms[j].translation.z);
	//         RCLCPP_INFO(rclcpp::get_logger("print_trajectory"), "  Tf rot: %f %f %f %f ", point.transforms[j].rotation.x,point.transforms[j].rotation.y,point.transforms[j].rotation.z,point.transforms[j].rotation.w);

	//     }
	// }

	std::string planning_frame = m_movegroupInterface->getPlanningFrame();
	std::cout<< "planning_frame "<< planning_frame<<std::endl;
	std::cout<< "trajectory_frame "<< trajectory.joint_trajectory.header.frame_id <<std::endl;
	std::cout<< "joint_trajectory size " << trajectory.joint_trajectory.points.size() << std::endl;


}

void MoveitInterface::writeTrajectoryToPickle(const moveit_msgs::msg::RobotTrajectory &trajectory){
	// Convert RobotTrajectory message to a dictionary
	YAML::Node trajectory_dict;
	trajectory_dict["joint_names"] = trajectory.joint_trajectory.joint_names;
	trajectory_dict["points"] = YAML::Load("[]");

	for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
	{
		YAML::Node point;
		point["positions"] = trajectory.joint_trajectory.points[i].positions;
		point["velocities"] = trajectory.joint_trajectory.points[i].velocities;
		point["accelerations"] = trajectory.joint_trajectory.points[i].accelerations;
		point["effort"] = trajectory.joint_trajectory.points[i].effort;
		// point["time_from_start"] = trajectory.joint_trajectory.points[i].time_from_start.toSec();
		trajectory_dict["points"].push_back(point);
	}

	// Write dictionary to .pkl file
	std::ofstream file("trajectory.pkl", std::ios::binary);
	// boost::archive::binary_oarchive oa(file);
	file << trajectory_dict;
	file.close();

	RCLCPP_INFO(rclcpp::get_logger("save_trajectory"), "Trajectory written to trajectory.pkl"); 

}


bool MoveitInterface::createWaypointTrajectory(std::vector<geometry_msgs::msg::Pose>& waypointVector)
{
	std::vector<geometry_msgs::msg::Pose> waypoints;
	rclcpp::Time time_now = this->get_clock()->now();
	rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.5);

	auto current_pose = m_movegroupInterface->getCurrentPose();

	waypoints.push_back(current_pose.pose);

	//REMOVE
	for(auto waypoint : waypointVector)
	{
	// 	geometry_msgs::msg::Pose output_wp_world;
		// getTf(waypoint,output_wp_world,"world","base");
		// geometry_msgs::msg::TransformStamped tfstamped = m_tfBuffer.lookupTransform("world","base", this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
		// tf2::doTransform(waypoint, output_wp_world, tfstamped);

		// waypoints.push_back(output_wp_world);
		waypoints.push_back(waypoint);
	}
	// waypoints.push_back(current_pose.pose);

	std::cout << "Waypoints:" << std::endl;
	for (size_t i = 0; i < waypoints.size(); ++i) {
		const geometry_msgs::msg::Pose& pose = waypoints[i];
		std::cout << "  - Position: (" << pose.position.x << ", "
			<< pose.position.y << ", " << pose.position.z << ")" << std::endl;

		std::cout << "    - Orientation (quaternion):" << std::endl;
		std::cout << "      - w: " << pose.orientation.w << std::endl;
		std::cout << "      - x: " << pose.orientation.x << std::endl;
		std::cout << "      - y: " << pose.orientation.y << std::endl;
		std::cout << "      - z: " << pose.orientation.z << std::endl;
	}

	// doTask(waypointVector);
	RCLCPP_INFO(this->get_logger(),"TASK");
	// double fraction = 0.0;
	// RCLCPP_INFO(this->get_logger(),"BEFORE!!!! Cartesian frajectory fraction value: %f", fraction);

	// while(fraction <= 0.75)
	// {
	//   fraction = m_movegroupInterface->computeCartesianPath(waypoints, 0.001, 0.0, m_robotTrajectory);
	//   RCLCPP_INFO(this->get_logger(),"Cartesian frajectory fraction value: %f", fraction);
	// }

	// printRobotTrajectory(m_robotTrajectory);
	// // Call write to pickle function
	// writeTrajectoryToPickle(m_robotTrajectory);

	// if(fraction > 0.0)
	return true;

}


bool MoveitInterface::createWaypointTrajectory(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypointVector)
{
	std::cout << "createWaypointTrajectory" << std::endl;
	std::vector<geometry_msgs::msg::Pose> waypoints;
	rclcpp::Time time_now = this->get_clock()->now();
	rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.5);

	// auto current_pose = m_movegroupInterface->getCurrentPose();
	// waypointVector["current_pose"] = current_pose.pose;

	RCLCPP_INFO(this->get_logger(),"TASK");
	bool taskStatus = false;
	taskStatus = doTask(current_task, waypointVector);
	return taskStatus;
}

void MoveitInterface::planTrajectory(bool &validTrajectory)
{
	if (validTrajectory)
	{
		// Retime the trajectory to apply velocity/acceleration scaling
		robot_trajectory::RobotTrajectory rt(
				m_movegroupInterface->getRobotModel(), m_movegroupInterface->getName());

		rt.setRobotTrajectoryMsg(*m_movegroupInterface->getCurrentState(), m_robotTrajectory);

		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		bool success = iptp.computeTimeStamps(
				rt, m_maxVel, m_maxAcc); //rt, vel_scaling, acc_scaling

		if (!success)
		{
			RCLCPP_ERROR(this->get_logger(), "Time parameterization failed!");
		}

		// Convert back to message
		moveit_msgs::msg::RobotTrajectory retimed_trajectory;
		rt.getRobotTrajectoryMsg(retimed_trajectory);

		// Create plan
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = retimed_trajectory;

		// Print Final Joint Positions
		const auto& final_joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
		RCLCPP_INFO(this->get_logger(), "Final Joint Positions:");
		for (size_t i = 0; i < final_joint_values.size(); ++i) {
			RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i + 1, final_joint_values[i]);
		}

		// Execute
		if(m_config["execute"].as<bool>())
		{
			RCLCPP_ERROR(this->get_logger(), "Executing!!!!");
			m_movegroupInterface->execute(plan);
		}
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Cartesian planning failed!");
	}
	m_completed = false;
}

void MoveitInterface::setParams()
{
	auto mg = m_config["move_group"];
	if (!mg) {
		std::cerr << "Move group config missing in YAML!" << std::endl;
		return;
	}
	std::string planner_id = m_config["move_group"]["planner_id"].as<std::string>();
	double planning_time = m_config["move_group"]["planning_time"].as<double>();
	int num_attempts = m_config["move_group"]["num_planning_attempts"].as<int>();
	m_maxVel = m_config["move_group"]["max_velocity_scaling_factor"].as<double>();
	m_maxAcc = m_config["move_group"]["max_acceleration_scaling_factor"].as<double>();
	std::string ee_link = m_config["move_group"]["end_effector_link"].as<std::string>();
	std::string pose_ref = m_config["move_group"]["pose_reference_frame"].as<std::string>();

	std::cout << "Loaded Move Group Config:" << std::endl;
	std::cout << "Planner ID: " << planner_id << std::endl;
	std::cout << "Planning Time: " << planning_time << std::endl;
	std::cout << "Num Planning Attempts: " << num_attempts << std::endl;
	std::cout << "Max Velocity Scaling Factor: " << m_maxVel << std::endl;
	std::cout << "Max Acceleration Scaling Factor: " << m_maxAcc << std::endl;
	std::cout << "End Effector Link: " << ee_link << std::endl;
	std::cout << "Pose Reference Frame: " << pose_ref << std::endl;

	if (!m_movegroupInterface) {
		RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized!");
		return; // or handle appropriately
	}
	m_movegroupInterface->setPlannerId(planner_id);
	m_movegroupInterface->setPlanningTime(planning_time);
	m_movegroupInterface->setNumPlanningAttempts(num_attempts);
	m_movegroupInterface->setMaxVelocityScalingFactor(m_maxVel);
	m_movegroupInterface->setMaxAccelerationScalingFactor(m_maxAcc);
	m_movegroupInterface->setEndEffectorLink(ee_link);
	m_movegroupInterface->setPoseReferenceFrame(pose_ref);
}


void MoveitInterface::isTransformAvailable(geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose, geometry_msgs::msg::TransformStamped& tfstamped, const std::string& target_frame, const std::string& source_frame, double timeout_sec)
{
	m_tfFound = false;
	rclcpp::Time time_now = this->get_clock()->now();
	rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);
	try 
	{
		tfstamped = m_tfBuffer.lookupTransform(target_frame, source_frame, time_now, rclcpp::Duration::from_seconds(0.5));
		tf2::doTransform(input_pose, output_pose, tfstamped);
		m_tfFound = true;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
		m_tfFound = false;
	}

}


void MoveitInterface::gripperService(bool& state)
{
	// m_waiting_for_gripper_response = false;
  if (m_waiting_for_gripper_response) {
    RCLCPP_WARN(this->get_logger(), "Gripper Service call in progress, skipping new request");
    return;
  }
  if (!m_gripperSrv->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Gripper Service not available yet");
    return;
  }

  auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
	if(state) //Open
	{
		request->position = 255;
		request->speed = 255;
		request->force = 128;	
	}
	else //Close
	{
		request->position = 0;
		request->speed = 64;
		request->force = 1;
	}
	
	m_waiting_for_gripper_response = true;

  auto future = m_gripperSrv->async_send_request(
    request,
    [this](rclcpp::Client<gripper_srv::srv::GripperService>::SharedFuture future) {
      auto response = future.get();
      if (response->response == "Done") {
        RCLCPP_INFO(this->get_logger(), "Service succeeded: %s", response->response.c_str());
		// m_waiting_for_gripper_response = true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->response.c_str());
      }
      m_waiting_for_gripper_response = false;
    }
  );
}


void MoveitInterface::callTriggerService(const std::string& key)
{
	if (m_waiting_for_response) {
		RCLCPP_WARN(this->get_logger(), "[%s] Waiting for previous service response.", key.c_str());
		return;
	}

	if (m_service_map.find(key) == m_service_map.end()) {
		RCLCPP_ERROR(this->get_logger(), "Service key [%s] not found.", key.c_str());
		return;
	}

	auto& info = m_service_map[key];

	if (!info.client->wait_for_service(std::chrono::seconds(1))) {
		RCLCPP_WARN(this->get_logger(), "[%s] Service not available", key.c_str());
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
			RCLCPP_INFO(this->get_logger(), "[%s] Service succeeded: %s", key.c_str(), response->message.c_str());
		} else {
			RCLCPP_ERROR(this->get_logger(), "[%s] Service failed: %s", key.c_str(), response->message.c_str());
		}
	}
	catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "[%s] Exception: %s", key.c_str(), e.what());
	}
		m_waiting_for_response = false;
	}
	);
}


void MoveitInterface::getTf(geometry_msgs::msg::Pose& input_pose,geometry_msgs::msg::Pose& output_pose,geometry_msgs::msg::TransformStamped& tfstamped, std::string target_frame,std::string source_frame)
{
	isTransformAvailable(input_pose, output_pose, tfstamped, target_frame, source_frame,0.5);
	if (m_tfFound)
	{
		// RCLCPP_INFO(this->get_logger(), "Path Transform verified.");
		m_tfFound = false;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Path Transform not found.");
	}
}


void MoveitInterface::loadCSVToPoses(const std::string& filename, std::vector<geometry_msgs::msg::Pose>& dummyPath)
{
	std::ifstream file(filename);
	std::string line;

	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::string value;
		std::vector<double> data;

		// Parse comma-separated values into the `data` vector
		while (std::getline(ss, value, ',')) {

			// Trim whitespace
			value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

			// Skip if value is empty
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
			// std::cout << "pose converted !" << std::endl;
		} else {
			std::cerr << "Skipping malformed line: " << line << std::endl;
		}
	}
}


geometry_msgs::msg::Pose MoveitInterface::lookupPoseTransformStamped(std::string target_frame_id, std::string source_frame_id)
{
	geometry_msgs::msg::Pose tfPose;
	try
	{
		geometry_msgs::msg::TransformStamped tfstamped = m_tfBuffer.lookupTransform(target_frame_id, source_frame_id, this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
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

bool MoveitInterface::generateStaticTFPose()
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
		m_screenRIght = lookupPoseTransformStamped(target_frame, source_frame);
		m_screenRIght.orientation = m_screenAlignPose.orientation;
		m_transformedPoses[source_frame] = m_screenRIght;

		loadCSVToPoses("/home/atu-2/robothon/src/moveit_interface/config/maze_path.csv",m_mazePath);

		return true;
	}
	catch(const tf2::TransformException & ex)
	{
		RCLCPP_INFO(this->get_logger(), "Could not transform 'base_link' to 'screen': %s", ex.what());
		return  false;
	}
}

void MoveitInterface::reset()
{
	m_screenTaskCounter = 0;
	m_frameStatus = false;
	m_mazePath.clear();
	m_mazePath.resize(0);
	m_detectionPoses.clear();
	m_detectionPoses.resize(0);
	m_callShapeService = false;
	m_callTextService = false;
	// m_service_map["localize_board"].srv_response.success = false;

}

void MoveitInterface::run()
{
	// m_movegroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "ur_manipulator");
	// m_state = InterfaceState::IDLE;
	if(m_config["custom_task"].as<bool>())
	{
		m_nextTaskType =  TaskType::BYOD;
		m_state = InterfaceState::EXECUTE;
	}

	m_completed = true;
	while (rclcpp::ok())
	{
		if(!m_completed)
		{
			switch (m_state)
			{
				case InterfaceState::IDLE:
					RCLCPP_INFO(this->get_logger(), "State: IDLE -> BOARD_DETECTION");
					m_state = InterfaceState::BOARD_DETECTION;
					break;

				case InterfaceState::BOARD_DETECTION:
					{
						RCLCPP_INFO(this->get_logger(), "State: BOARD_DETECTION -> WAIT_FOR_RESPONSE");
						if(!m_service_map["localize_board"].srv_response.success)
						{
							callTriggerService("localize_board");
						}
						else
							m_state = InterfaceState::WAIT_FOR_RESPONSE;
						//std::cout << "m_waiting_for_response " << m_waiting_for_response << std::endl;
						break;
					}
				case InterfaceState::WAIT_FOR_RESPONSE:
					{
						RCLCPP_INFO(this->get_logger(), "State: WAIT_FOR_RESPONSE -> CHECK_TF");
						if (m_frameStatus) {
							RCLCPP_INFO(this->get_logger(), "Frame received. Proceeding to TF check.");
							m_service_map["localize_board"].srv_response.success = false;
							m_waiting_for_response = false;
							m_state = InterfaceState::CHECK_TF;
						}
						break;
					}
				case InterfaceState::CHECK_TF:
					{
						RCLCPP_INFO(this->get_logger(), "State: CHECK_TF -> EXECUTE");
						m_movegroupInterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "ur_manipulator");
						setParams();
						bool state = generateStaticTFPose();
						if(state)
						{
							RCLCPP_INFO(this->get_logger(), "TF Lookup successful. Ready to proceed.");
							bool gripper_state = true;
							gripperService(gripper_state); //Open gripper
							m_state = InterfaceState::EXECUTE;
							m_nextTaskType =  TaskType::SPEED_PRESS; //start SPEED_PRESS
						}
						else
						{
							RCLCPP_INFO(this->get_logger(), "TF Lookup unsuccessful. retrying");
						}
						break;
					}
				case InterfaceState::EXECUTE:
					{
						RCLCPP_WARN(this->get_logger(), "State: EXECUTE -> DONE");
						// RCLCPP_WARN(this->get_logger(), "Proceeding to final task (e.g., planning)...");
						m_waiting_for_response = false; //To ensure all states are called freshly
						executeTasks(m_nextTaskType);
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

void MoveitInterface::executeTasks(TaskType& task)
{
	switch(task)
	{
		case TaskType::NONE:
		{
			RCLCPP_ERROR(this->get_logger(),"No task type Defined, executing default");
			m_taskType = TaskType::END;
			m_state = InterfaceState::DONE;
			break;
		}

		case TaskType::SPEED_PRESS:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: SPEED_PRESS");
			bool state = executeSpeedPress();
			if(state)
				m_nextTaskType = TaskType::PRESS_BUTTONS;
			break;
		}

		case TaskType::PRESS_BUTTONS:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: PRESS_BUTTONS");
			bool state = executeButtonPress();
			if(state)
				m_nextTaskType = TaskType::GRAB_STYLUS_TOUCH;
			break;
		}

		case TaskType::GRAB_STYLUS_TOUCH:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: GRAB_STYLUS_TOUCH");
			bool state = executeGrabStylus(task);
			if(state)
				m_nextTaskType = TaskType::GRAB_STYLUS_MAGNET;
			break;
		}

		case TaskType::SCREEN_SHAPE:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: SCREEN_SHAPE");
			bool state = executeScreenMotion();
			if(state)
				m_nextTaskType = TaskType::SCREEN_TEXT;
			break;
		}

		case TaskType::SCREEN_TEXT:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: SCREEN_TEXT");
			bool state = executeScreenText();
			if(state)
				m_nextTaskType = TaskType::MAZE;
			break;
		}

		case TaskType::GRAB_STYLUS_MAGNET:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: GRAB_STYLUS_MAGNET");
			bool state = executeGrabStylus(task);
			if(state)
				m_nextTaskType = TaskType::SCREEN_SHAPE;
			break;
		}

		case TaskType::MAZE:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: MAZE");
			bool state = executeMaze();
			if(state)
				m_nextTaskType = TaskType::DROP_STYLUS;
			break;
		}
		case TaskType::DROP_STYLUS:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: DROP_STYLUS");
			bool state = executeDropStylus();
			if(state)
				m_nextTaskType = TaskType::END;
			break;
		}
		case TaskType::BYOD:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: BYOD");
			bool state = executeCustomTask();
			if(state)
				m_nextTaskType = TaskType::END;
			break;
		}
		case TaskType::END:
		{
			RCLCPP_ERROR(this->get_logger(),"TaskType: END");
			reset();
			m_state = InterfaceState::DONE;
			break;
		}

	}
}

bool MoveitInterface::executeMaze()
{
	std::string target_frame = "base_link";
	std::string source_frame = "blue_button";
	std::string current_task = "solve_maze";

	try
	{
		geometry_msgs::msg::TransformStamped tfstamped;
		tfstamped.transform.translation.x = m_transformedPoses[source_frame].position.x;
		tfstamped.transform.translation.y = m_transformedPoses[source_frame].position.y;
		tfstamped.transform.translation.z = m_transformedPoses[source_frame].position.z;
		tfstamped.transform.rotation.x = m_transformedPoses[source_frame].orientation.x;
		tfstamped.transform.rotation.y = m_transformedPoses[source_frame].orientation.y;
		tfstamped.transform.rotation.z = m_transformedPoses[source_frame].orientation.z;
		tfstamped.transform.rotation.w = m_transformedPoses[source_frame].orientation.w;

		// 
		//  = m_tfBuffer.lookupTransform(target_frame, source_frame, this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
		std::cout << "m_mazePath " << m_mazePath.size() << std::endl;

		std::map<std::string, geometry_msgs::msg::Pose> named_poses;
		named_poses["screen_align_pose"] = m_transformedPoses["align_frame"];
		int index = 10;
		for(auto pose : m_mazePath)
		{
			geometry_msgs::msg::Pose transformed_maze_pose;
			// getTf(pose,transformed_maze_pose,tfstamped,target_frame,source_frame);
			tf2::doTransform(pose, transformed_maze_pose, tfstamped);
			transformed_maze_pose.orientation = m_transformedPoses["align_frame"].orientation;
			std::string name = "maze_pathpoint_" + std::to_string(index);
			RCLCPP_INFO(this->get_logger(),"nameeeee %s", name.c_str());
			named_poses[name] = transformed_maze_pose;
			index++;
		}

		bool validTrajectory = false;
		validTrajectory = createWaypointTrajectory(current_task, named_poses);
		if(validTrajectory)
		{
			current_task = "retract_maze";
			validTrajectory = createWaypointTrajectory(current_task, named_poses);
			return validTrajectory;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Could not execute maze task");
			return false;
		}
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
}


bool MoveitInterface::executeDropStylus()
{
	std::string current_task = "place_stylus"; 

	std::map<std::string, geometry_msgs::msg::Pose> named_poses;

	named_poses["stylus_pose"] = m_transformedPoses["stylus"];

	bool validTrajectory = false;
	validTrajectory = createWaypointTrajectory(current_task, named_poses);

	if(validTrajectory)
	{
		bool gripper_state = false;
		gripperService(gripper_state);
		current_task = "retract_stylus";
		validTrajectory = createWaypointTrajectory(current_task, named_poses);
		current_task = "home_pose";
		validTrajectory = createWaypointTrajectory(current_task, named_poses);
		return validTrajectory;
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Could not reach stylus pose");
		return false;
	}
}

bool MoveitInterface::executeGrabStylus(TaskType& taskType)
{
	std::string current_task = ""; 
	if(taskType == TaskType::GRAB_STYLUS_TOUCH)
	{	
		bool gripper_state = false;
		bool serviceRequested = false;

		gripperService(gripper_state); //Open gripper
	
		std::string current_task = "pick_stylus"; 
	
		std::map<std::string, geometry_msgs::msg::Pose> named_poses;

		named_poses["stylus_pose"] = m_transformedPoses["stylus"];

		bool validTrajectory = false;
		validTrajectory = createWaypointTrajectory(current_task, named_poses);

		if(validTrajectory)
		{
			
			gripper_state = true;
			gripperService(gripper_state);
			sleep(1);
			RCLCPP_INFO(this->get_logger(),"Waiting done");
			current_task = "retract_stylus";
			validTrajectory = createWaypointTrajectory(current_task, named_poses);
			return validTrajectory;
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Could not reach stylus pose");
			return false;
		}
	}
	else if(taskType == TaskType::GRAB_STYLUS_MAGNET)
	{
		current_task = "retract_stylus_invert"; 
		geometry_msgs::msg::TransformStamped tfstamped = m_tfBuffer.lookupTransform("base_link", "stylus_calibration", this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
		geometry_msgs::msg::Pose calibPose;
		calibPose.position.x = tfstamped.transform.translation.x;
		calibPose.position.y = tfstamped.transform.translation.y;
		calibPose.position.z = tfstamped.transform.translation.z;
		calibPose.orientation.x = tfstamped.transform.rotation.x;
		calibPose.orientation.y = tfstamped.transform.rotation.y;
		calibPose.orientation.z = tfstamped.transform.rotation.z;
		calibPose.orientation.w = tfstamped.transform.rotation.w;

		std::map<std::string, geometry_msgs::msg::Pose> named_poses;

		named_poses["stylus_calibration_precise"] = calibPose;

		bool validTrajectory = false;
		validTrajectory = createWaypointTrajectory(current_task, named_poses);
		return validTrajectory;

	}

}


bool MoveitInterface::executeScreenText()
{
	std::string target_frame = "";
	std::string source_frame = "";
	std::string current_task = "";

	//@NOTE: Uncomment if testing the screen text task independently - go to screen approach first
	// if(!m_callTextService) 
	// {

	// 	current_task = "screen_approach";
	// 	std::map<std::string, geometry_msgs::msg::Pose> named_poses;
	// 	named_poses["screen_align_pose"] = m_transformedPoses["align_frame"];
	// 	named_poses["screen"] = m_transformedPoses["screen"];

	// 	bool validTrajectory = false;
	// 	validTrajectory = createWaypointTrajectory(current_task, named_poses);
	// 	m_callTextService = validTrajectory;
	// 	return false;
	// }
	// else
	{
		if(!m_service_map["detect_text"].srv_response.success)// && m_srvAttempts < 3)
		{
			callTriggerService("detect_text");
			// m_srvAttempts++;
			return false;
		}

		if (m_screenCommand == "")
		{
			RCLCPP_INFO(this->get_logger(),"Waiting for m_screenCommand");
			return false;
		}
			
		current_task = "screen_text";
		target_frame = "base_link";
		
        ParsedTask parsedTask;
		parseTaskCommand(m_screenCommand,parsedTask);
		std::map<std::string, geometry_msgs::msg::Pose> named_poses;

		int repeat_count = 1;

		std::cout << "ID: " << parsedTask.id << " size " <<parsedTask.task_names.size() << "\n";
		for (const auto& name : parsedTask.task_names)
		{
			
			std::cout << "Task from screen : " << name << "\n";
			source_frame = name;
			geometry_msgs::msg::Pose targetPose;
			try
			{
				targetPose = m_transformedPoses[name];
				named_poses[name] = targetPose;
				std::cout << targetPose.position.x << " " << targetPose.position.y << std::endl;
			}
			catch (const tf2::TransformException & ex) {
				RCLCPP_INFO(this->get_logger(), "Could not transform 'base_link' to 'point on screen: %s", ex.what());
				return  false;
			}

		}

		named_poses["screen"] = m_transformedPoses["screen"];


		if(m_config["planning"].as<bool>())
		{
			bool validTrajectory = false;
			validTrajectory = createWaypointTrajectory(current_task, named_poses);
			if(validTrajectory && m_screenTaskCounter < 2)
			{
				m_screenTaskCounter++;
				m_service_map["detect_text"].srv_response.success = false;
				return false;
			}
			else
			{
				current_task = "retract_align_screen";
				bool homepose = createWaypointTrajectory(current_task, named_poses);
				m_callTextService = false;
				m_taskType = TaskType::END;
				return true;
			}
		}
	}
}

bool MoveitInterface::executeScreenMotion()
{
	std::string target_frame = "";
	std::string source_frame = "";
	std::string current_task = "";

	std::cout << "m_callShapeService " << m_callShapeService << std::endl;
	if(!m_callShapeService)
	{
		current_task = "screen_approach";
		source_frame = "align_frame";

		std::map<std::string, geometry_msgs::msg::Pose> named_poses;
		named_poses["screen_align_pose"] = m_transformedPoses["align_frame"];
		named_poses["screen"] = m_transformedPoses["screen"];
		
		bool validTrajectory = false;
		validTrajectory = createWaypointTrajectory(current_task, named_poses);
		m_callShapeService = validTrajectory;
		return false;
	}
	else
	{
		std::cout << "m_labelData " << m_labelData << std::endl;
		if(!m_service_map["detect_shape"].srv_response.success) // || m_labelData == "None")
		{
			callTriggerService("detect_shape");
			return false;
		}

		if (m_shapePoses.poses.empty())
		{
			RCLCPP_INFO(this->get_logger(),"Waiting for detection");
			// m_service_map["detect_shape"].srv_response.success = false;
			return false;
		}
			
		std::cout << "Shape Detected\n";
		current_task = "screen_draw";
		target_frame = "base_link";
		source_frame = m_shapePoses.header.frame_id;

		try
		{
			geometry_msgs::msg::TransformStamped tfstamped = m_tfBuffer.lookupTransform(target_frame, source_frame, this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));

			std::map<std::string, geometry_msgs::msg::Pose> named_poses;
			named_poses["background"] = m_transformedPoses["Background"];
			int index = 0;
			for(auto pose : m_shapePoses.poses)
			{
				geometry_msgs::msg::Pose temp;
				getTf(pose,temp,tfstamped,target_frame,source_frame);
				std::cout << " pose " << pose.position.x << " " << pose.position.y << std::endl; 
				std::cout << " temp " << temp.position.x << " " << temp.position.y << std::endl; 
				temp.position.z = m_transformedPoses["Background"].position.z;
				std::string name = "screen_draw" + std::to_string(index);
				named_poses[name] = temp;
				index++;
			}

			named_poses["screen"] = m_transformedPoses["screen"];

			bool validTrajectory = false;
			validTrajectory = createWaypointTrajectory(current_task, named_poses);
			if(validTrajectory && m_screenTaskCounter < m_maxAttempt)
			{
				m_screenTaskCounter++;
				m_service_map["detect_shape"].srv_response.success = false;
				return false;
			}
			else
			{
				// current_task = "home_pose";
				// bool homepose = createWaypointTrajectory(current_task, named_poses);
				m_screenTaskCounter = 0;
				m_callShapeService = false;
				// m_service_map["detect_shape"].srv_response.success = false;

				m_taskType = TaskType::END;
				return true;
			}
		}
		catch (const tf2::TransformException & ex) {
			RCLCPP_INFO(this->get_logger(), "Could not transform 'base_link' to 'camera': %s", ex.what());
			return  false;
		}
	}
}


void MoveitInterface::parseTaskCommand(std::string& taskCommand,ParsedTask& parsedTask)
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
				std::cout << "Adding task: " << task_name << " [repeat " << (j + 1) << "]\n";
				parsedTask.task_names.push_back(task_name); 	
			}
	}
}
}


bool MoveitInterface::executeSpeedPress()
{
	std::cout << "executeSpeedPress" << std::endl;

	std::string target_frame = "";
	std::string source_frame = "";
	std::string current_task = "speed_test"; 
	
	std::map<std::string, geometry_msgs::msg::Pose> named_poses;
	named_poses["blue_button"] = m_transformedPoses["blue_button"];
	named_poses["red_button"] = m_transformedPoses["red_button"];

	bool validTrajectory = false;
	validTrajectory = createWaypointTrajectory(current_task, named_poses);
	m_taskType = TaskType::END;
	return validTrajectory;

}



bool MoveitInterface::executeButtonPress()
{
	std::string current_task = "";
	if(m_service_map["detect_button"].srv_response.success)
	{
		if(m_buttonStatus != "None")
		{
			std::map<std::string, geometry_msgs::msg::Pose> named_poses;
			// std::cout << "Button Detected\n";
			if(m_buttonStatus == "Red")
			{
				current_task = "press_red_button";
				named_poses["red_button"] = m_transformedPoses["red_button"];

			}
			else
			{
				current_task = "press_blue_button";
				named_poses["blue_button"] = m_transformedPoses["blue_button"];;

			}

			bool validTrajectory = false;
			validTrajectory = createWaypointTrajectory(current_task, named_poses);
			m_service_map["detect_button"].srv_response.success = false;
			m_taskType = TaskType::END;
			return validTrajectory;
		}
		else
		{
			// std::cout << "Waiting for detection "  << m_buttonStatus << "\n";
			return false;
		}
	}
	else
	{
		callTriggerService("detect_button");
		return false;
	}
}

bool MoveitInterface::executeCustomTask()
{
	std::string target_frame = "";
	std::string source_frame = "";
	std::string current_task = "";

	{
		std::cout << " stat " << m_service_map["color_sort"].srv_response.success << std::endl;
		if(!m_service_map["color_sort"].srv_response.success)
		{
			callTriggerService("color_sort");
			return false;
		}

		if (m_detectionPoses.empty())
		{
			RCLCPP_INFO(this->get_logger(),"Waiting for detection");
			// m_service_map["color_sort"].srv_response.success = false;
			return false;
		}
			
		if (m_screenCommand == "")
		{
			RCLCPP_INFO(this->get_logger(),"Waiting for m_screenCommand");
			return false;
		}

		current_task = m_screenCommand; //Input from topic
		target_frame = "base_link";
		source_frame = "camera_color_optical_frame";

		std::map<std::string, std::vector<geometry_msgs::msg::Pose>> sortingPoses;

		try
		{
			RCLCPP_INFO(this->get_logger(), "%d ",m_detectionPoses.size());
			geometry_msgs::msg::TransformStamped tfstamped = m_tfBuffer.lookupTransform(target_frame, source_frame, this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));
			geometry_msgs::msg::TransformStamped tfstamped_gripper = m_tfBuffer.lookupTransform(target_frame, "ee_touch", this->get_clock()->now(), rclcpp::Duration::from_seconds(0.5));

			std::map<std::string, geometry_msgs::msg::Pose> named_poses;
			// int index = 0; //Lexographical comparison to be handled further
			for(auto detection_array : m_detectionPoses)
			{
				std::string name = detection_array.header.frame_id + "_object";
				for (auto pose : detection_array.poses)
				{
					geometry_msgs::msg::Pose objPose;
					getTf(pose,objPose,tfstamped,target_frame,source_frame);
					objPose.orientation = tfstamped_gripper.transform.rotation;
					sortingPoses[name].push_back(objPose);
				}
				// index++;
			}

			RCLCPP_INFO(this->get_logger(), "%d ",sortingPoses.size());

			for (const auto& [bin, poses] : sortingPoses) {
				RCLCPP_INFO(this->get_logger(), "Bin: %s", bin.c_str());
				for (const auto& pose : poses) {
					RCLCPP_INFO(this->get_logger(), "  x: %.2f y: %.2f z: %.2f",
								pose.position.x, pose.position.y, pose.position.z);
				}
			}


			for (const auto& [key, pose_vector] : sortingPoses) {
				if (!pose_vector.empty()) {
					RCLCPP_INFO(this->get_logger(), "key: %s", key.c_str());
					named_poses[key] = pose_vector[0];  // or whatever logic you want
				}
			}
			
			bool validTrajectory = false;
			validTrajectory = createWaypointTrajectory(current_task, named_poses);
			m_service_map["color_sort"].srv_response.success = false;
			return validTrajectory;
		}
		catch (const tf2::TransformException & ex) {
			RCLCPP_INFO(this->get_logger(), "Could not transform 'base_link' to 'camera': %s", ex.what());
			return  false;
		}
	}
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

	// moveit::planning_interface::PlanningSceneInterface psi2;
	scene_.applyCollisionObject(object2);

}

bool MoveitInterface::doTask(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypoints)
{
	RCLCPP_INFO(this->get_logger(),"Create task");
	task_ = createTask(current_task, waypoints);

	try
	{
		task_.init();
	}
	catch (mtc::InitStageException& e)
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), e);
		return false;
	}

	if (!task_.plan(15))
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), "Task planning failed");
		return false;
	}
	task_.introspection().publishSolution(*task_.solutions().front());

	auto result = task_.execute(*task_.solutions().front());
	if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), "Task execution failed");
		return false;
	}

	return true;
}

void MoveitInterface::addStagesFromYaml(mtc::Task& task, const YAML::Node& task_config, const std::map<std::string, geometry_msgs::msg::Pose>& named_poses, std::map<std::string, mtc::solvers::PlannerInterfacePtr>& planners, const std::string& group_name, const std::string& hand_frame)
{

	RCLCPP_INFO(this->get_logger(),"addStagesFromYaml");
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
		if(stage_node["hand_frame"]) { hand_frame_name = stage_node["hand_frame"].as<std::string>(); }
		else hand_frame_name = hand_frame;
		
		if(stage_node["minmax_dist"])
		{
			min_distance = stage_node["minmax_dist"][0].as<double>();
			max_distance = stage_node["minmax_dist"][1].as<double>();
		}
		
		if(stage_node["vel_acc"])
		{
			planner->setMaxVelocityScalingFactor(stage_node["vel_acc"][0].as<double>());
			planner->setMaxAccelerationScalingFactor(stage_node["vel_acc"][1].as<double>());
		}

		RCLCPP_INFO(this->get_logger(),"TASK TYPE %s",type.c_str());

		if(type == "move_to")
		{
			std::string target_name = stage_node["target"].as<std::string>();
			auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);

			stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
			stage->setGroup(group_name);
			stage->setIKFrame(hand_frame);

			std::cout << target_name << std::endl;
			if(target_name != "home_camera_vertical" && target_name != "home_camera" && target_name != "home_camera_touch" && target_name != "home_camera_magnet" && target_name != "stylus_calibration")
			{
				auto pose = named_poses.at(target_name);
				auto offset_vals = stage_node["offset"];
				geometry_msgs::msg::PoseStamped offset_pose;
				offset_pose.header.frame_id = "base_link";
				offset_pose.pose = pose;
				offset_pose.pose.position.x = offset_pose.pose.position.x + offset_vals[0].as<double>();
				offset_pose.pose.position.y = offset_pose.pose.position.y + offset_vals[1].as<double>();
				offset_pose.pose.position.z = offset_pose.pose.position.z + offset_vals[2].as<double>();
				stage->setGoal(offset_pose);
			}
			else if(target_name == "home_camera") //Requires string parse from srdf
			{
				stage->setGoal("home_camera");
			}
			else if(target_name == "home_camera_touch")
			{
				stage->setGoal("home_camera_touch");
			}
			else if(target_name == "home_camera_magnet")
			{
				stage->setGoal("home_camera_magnet");
			}
			else if(target_name == "stylus_calibration")
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
			// std::cout << "min max " << min_distance << " "  << max_distance << std::endl;
			auto dir_vals = stage_node["direction"];
			std::string frame_id = "world";
			if(stage_node["frame"])
			{
				std::cout << "Frame defined " << stage_node["frame"] << std::endl;
				frame_id = stage_node["frame"].as<std::string>();
			}
			geometry_msgs::msg::Vector3 dir;
			dir.x = dir_vals[0].as<double>();
			dir.y = dir_vals[1].as<double>();
			dir.z = dir_vals[2].as<double>();

			auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
			stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
			stage->setGroup(group_name);
			stage->setMinMaxDistance(min_distance, max_distance);  // Optional: load from YAML
			stage->setIKFrame(hand_frame);

			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = frame_id;
			vec.vector = dir;
			stage->setDirection(vec);

			task.add(std::move(stage));
		}
		else
		{
			RCLCPP_INFO(this->get_logger(),"Creating serial ");
			auto path = std::make_unique<mtc::SerialContainer>("follow path");
			task.properties().exposeTo(path->properties(), { "group", "ik_frame" });
			path->properties().configureInitFrom(mtc::Stage::PARENT,
													{ "group", "ik_frame" });
			
			std::string target_name = stage_node["target"].as<std::string>();
			int index = 0;
			for(auto& [path_point_name, path_point] : named_poses)
			{
				if(path_point_name == "screen" || path_point_name == "screen_align_pose")
					continue;
				auto stage = std::make_unique<mtc::stages::MoveTo>(path_point_name, planner);
				RCLCPP_INFO(this->get_logger(),"target %s path_point_name %s",target_name.c_str(),  path_point_name.c_str());
				stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
				stage->setGroup(group_name);
				stage->setIKFrame(hand_frame);

				auto pose = named_poses.at(path_point_name);
				auto offset_vals = stage_node["offset"];
				// geometry_msgs::msg::PoseStamped offset_pose;
				geometry_msgs::msg::PointStamped offset_pose;
				offset_pose.header.frame_id = "base_link";
				// offset_pose.pose = pose;
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

mtc::Task MoveitInterface::createTask(std::string& current_task, std::map<std::string, geometry_msgs::msg::Pose>& waypoints)
{
	mtc::Task task;
	task.stages()->setName(current_task);
	task.loadRobotModel(this->shared_from_this());

	std::string hand_frame_config = "ee_gripper";

	if(m_config["tasks"][current_task]["hand_frame"])
	{
		hand_frame_config = m_config["tasks"][current_task]["hand_frame"].as<std::string>();
		std::cout << "hand_frame_config " << hand_frame_config.c_str() << std::endl;
	}

	const auto& arm_group_name = "ur_manipulator";
	// const auto& hand_group_name = "hand";
	const auto& hand_frame = hand_frame_config;

	// Set task properties
	task.setProperty("group", arm_group_name);
	// task.setProperty("eef", hand_group_name);
	task.setProperty("ik_frame", hand_frame);

	
	// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

	auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
	current_state_ptr = stage_state_current.get();
	task.add(std::move(stage_state_current));

	auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
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
