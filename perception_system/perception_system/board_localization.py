import rclpy
import rclpy.logging
from rclpy.node import Node

from std_msgs.msg import String, Bool

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Transform, Vector3, TransformStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import geometry_msgs.msg
import datetime
from dataclasses import dataclass, field
from typing import List, Tuple
import torch
import numpy as np
import os, math
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
import cv2
from typing import List, Tuple, Optional
from scipy.spatial.transform import Rotation as R
import threading


# from slider_detect_triangles import *
from .inference import OnlineInference
from .realsense import RealSense

COLOR_FRAME_TOPIC = '/camera/color/image_raw'

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

SERVICE_CALLBACK = GREEN + "Service call {} received" + END
PARAM_NOT_DEFINED_ERROR = "Parameter: {} not defined"
SUCCESSFUL = "Successfully executed"
NOT_SUCCESSFUL = "Not Successfully executed"

SERVICE_NAME_BOARD = "/taskboard/board_localization"

SLIDER_TRAVEL_MM = 9.1  # 8.5 #29.0
SLIDER_TRAVEL_PX = 56  # 56  # 49 #256.0


# @dataclass
class BoardLocalization(Node):
    def __init__(self, realsense_node):
        # Initialize Node explicitly
        super().__init__('board_localization_node')

        self.declare_parameter('test', True)
        # self.declare_parameter('images_path', '')
        self.declare_parameter('weight_path', '')
        self.declare_parameter('labels_path', '')

        # Get parameters
        self.test = self.get_parameter('test').get_parameter_value().bool_value
        # self.images_path = self.get_parameter('images_path').get_parameter_value().string_value
        self.weight_path = self.get_parameter('weight_path').get_parameter_value().string_value
        self.labels_path = self.get_parameter('labels_path').get_parameter_value().string_value

        if not self.test:
            self.get_logger().error(RED + PARAM_NOT_DEFINED_ERROR.format("test") + END)
            rclpy.shutdown()
            return

        # if not self.images_path or not os.path.exists(self.images_path):
        #     self.get_logger().error(RED + "Images_path is not defined or does not exist" + END)
        #     rclpy.shutdown()
        #     return

        if not self.weight_path:
            self.get_logger().error(RED + PARAM_NOT_DEFINED_ERROR.format("weight_path") + END)
            rclpy.shutdown()
            return

        if not self.labels_path:
            self.get_logger().error(RED + PARAM_NOT_DEFINED_ERROR.format("labels_path") + END)
            rclpy.shutdown()
            return

        self.realsense: RealSense = field(init=False)
        self.broadcaster: tf2_ros.StaticTransformBroadcaster = field(init=False)
        self.tf_buffer: Buffer = field(init=False)
        self.tf_listener: TransformListener = field(init=False)
        self.img_size: List[int] = field(init=False, default_factory=lambda: [1280, 720])
        self.first_identification: bool = field(init=False, default=True)
        self.red_button_camera: np.array = field(init=False, default=None)
        self.blue_button_camera: np.array = field(init=False, default=None)
        self.n_frame: int = field(init=False, default=0)
        self.logging: bool = field(init=False, default=True)
        self.bridge: CvBridge = field(init=False)
        self.online_board_inference: OnlineInference = field(init=False)
        self.srv_execution: bool = field(init=False, default=False)

        self.status_publisher = self.create_publisher(Bool, 'frame_status', 10)


        # Create a ROS2 service
        self.srv = self.create_service(Trigger, SERVICE_NAME_BOARD, self.board_localization_callback)
        self.get_logger().info("Board localization service started.")
            

        # Initialize RealSense and get camera parameters
        self.realsense = realsense_node
        self.get_logger().info("Waiting camera parameters ...")
        self.realsense.getCameraParam()
        self.realsense.waitCameraInfo()
        self.get_logger().info("Camera parameters retrieved correctly")

        # Setup TF broadcaster and listener
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()

        self.online_board_inference = OnlineInference(
            weights=self.weight_path,
            device="cpu"
        )

        self.get_logger().info("Service alive...")


    def board_localization_callback(self, request, response):
        self.get_logger().info("Starting Board Localization...")

        # Start heavy work in a separate thread and return immediately
        threading.Thread(target=self.run_loop, args=(request,)).start()
        
        # Prepare and return immediate response
        # self.get_logger().info(f" {self.srv_execution} : Service")
        # if self.srv_execution:
        # self.get_logger().info("SRv True?")
        self.publish_status(False)
        response.success = True
        response.message = "Processing Started"
        return response
    
    def process_feature(self, feature_key, features, depth_frame, confidence_threshold=0.5):
        if features[feature_key]:
            self.get_logger().info("Okay")
        else:
            self.get_logger().error("Not Okay! BYE")
        if features[feature_key] and features[feature_key][2] >= confidence_threshold:
            u, v = features[feature_key][0], features[feature_key][1]
            self.logging and self.get_logger().info(GREEN + f"{feature_key.upper()}=> {u},{v}" + END)

            # Deproject and convert to meters
            feature_camera = np.array(
                self.realsense.deproject(u, v, depth_frame[v, u])
            ) / 1000.0

            attr_name = f"{feature_key}_camera"
            n_name = f"n_{feature_key}"

            prev_val = getattr(self, attr_name, None)
            count = getattr(self, n_name, 1)

            if prev_val is not None:
                new_val = prev_val + (1.0 / count) * (feature_camera - prev_val)
                setattr(self, attr_name, new_val)
                setattr(self, n_name, count + 1)
            else:
                setattr(self, attr_name, feature_camera)
                setattr(self, n_name, 1)

            self.logging and self.get_logger().info(
                f"Diff {feature_key}: {feature_camera - getattr(self, attr_name)}"
            )
            self.logging and self.get_logger().info(BOLD + f"count {count} and {feature_key} => {features[feature_key][0]},{features[feature_key][1]}"+ END)
            return count, feature_camera
        else:
            return 0,None

    def run_loop(self, request):
        # Do your blocking or long-running processing here
        self.get_logger().info('Loop started')
        try:
            self.srv_execution = False
            self.red_button_camera = None
            self.blue_button_camera = None

            self.screen = None
            self.maze = None
            self.task_board = None
            self.stylus = None
            red_cam = None
            blue_cam = None
                
            n_red, n_blue = 0, 0
            confidence_threshold = 0.70
            self.first_identification = True

            # rate = self.create_rate(10)  # 10 Hz

            while rclpy.ok() and self.first_identification:
                self.realsense.acquireOnceBoth()
                rgb_frame = self.realsense.getColorFrame()

                # self.n_frame += 1

                # if self.test:
                #     now = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
                #     self.realsense.saveAquiredImage(f"{self.images_path}frame_{now}.png")

                frame = rgb_frame
                features = {
                    "red_button": [],
                    "blue_button": [],
                    "maze": [],
                    "stylus": [],
                    "task_board": [],
                    "screen": []
                }

                if self.logging:
                    self.get_logger().info("---------------- BEFORE INFERENCE -----------")

                if frame is None:
                    self.get_logger().error("NO FRAME AVAILABLE")
                inference_result, _, _ = self.online_board_inference.realtime_inference(frame, view_img=False)

                if self.logging:
                    self.get_logger().info("---------------- INFERENCE RESULT -----------")
                    self.get_logger().info(str(inference_result))

                # Parse inference results, update feature centroids...
                for recognize_object in inference_result:
                    class_name = recognize_object["class_name"]
                    polygon = recognize_object.get("polygon")
                    conf = recognize_object["conf"]

                    if class_name in features:
                        centroid_x = int(np.mean([pt[0] for pt in polygon]))
                        centroid_y = int(np.mean([pt[1] for pt in polygon]))

                        if features[class_name]:  # If already exist take the one with biggest conf
                            if features[class_name][2] > conf:
                                continue
                        features[class_name] = [centroid_x, centroid_y, conf]
                self.logging and self.get_logger().info("========================")
                self.logging and print(features,flush=True)
                self.logging and self.get_logger().info("========================")

                depth_frame = self.realsense.getDistanceFrame()
                self.get_logger().info(f"Shape depth: {depth_frame.shape}")
            
                
                
                n_red, self.red_button_camera = self.process_feature("red_button",features,depth_frame,confidence_threshold)
                n_blue, self.blue_button_camera = self.process_feature("blue_button",features,depth_frame,confidence_threshold)
                _, self.screen = self.process_feature("screen",features,depth_frame,confidence_threshold)
                _, self.maze = self.process_feature("maze",features,depth_frame,confidence_threshold)
                _, self.task_board = self.process_feature("task_board",features,depth_frame,confidence_threshold)
                _, self.stylus = self.process_feature("stylus",features,depth_frame,confidence_threshold)

                if self.first_identification and n_red >= 1 and n_blue >= 1:
                    self.first_identification = False

            self.logging and self.get_logger().info("========== TRANSFORMATIONS ==========")
            self.logging and self.get_logger().info(f"Red -  {self.red_button_camera}")
            self.logging and self.get_logger().info(f"Blue - {self.blue_button_camera}")
            self.logging and self.get_logger().info(f"Screen - {self.screen}")
            self.logging and self.get_logger().info(f"Maze - {self.maze}")
            self.logging and self.get_logger().info(f"Stylus - {self.stylus}")
            self.logging and self.get_logger().info(f"Task Board - {self.task_board}")
            self.logging and self.get_logger().info("=====================================")


            if (self.red_button_camera is None) and (self.blue_button_camera is None):
                self.get_logger().info(RED + "Something not identified" + END)
                raise RuntimeError("Something not identified")
                # response = Trigger.Response()
                # response.success = False
                # response.message = NOT_SUCCESSFUL
                # return response

            else:
                self.logging and self.get_logger().info(YELLOW + f"Red button found: {n_red} and {features['red_button'][2]} " + END)
                self.logging and self.get_logger().info(YELLOW + f"Blue button found: {n_blue} and {features['blue_button'][2]}" + END)

            self.online_board_inference.realtime_inference(frame, view_img=True)

            # rate.sleep()
            found = False
            def translation_matrix(translation_vec):
                M = np.identity(4)
                M[:3, 3] = translation_vec
                return M
            
            def quaternion_matrix(quaternion):
                r = R.from_quat(quaternion)
                matrix = np.identity(4)
                matrix[:3, :3] = r.as_matrix()
                return matrix

            # Helper function similar to tf.transformations.quaternion_from_matrix
            def quaternion_from_matrix(matrix):
                r = R.from_matrix(matrix[:3, :3])
                return r.as_quat()

            # TF transform lookup example - wait until transform available
            while not found and rclpy.ok():
                try:
                    now = rclpy.time.Time()
                    trans_stamped = self.tf_buffer.lookup_transform(
                        'base_link',
                        'camera_color_optical_frame',
                        rclpy.time.Time())
                    trans = [
                        trans_stamped.transform.translation.x,
                        trans_stamped.transform.translation.y,
                        trans_stamped.transform.translation.z
                    ]
                    rot = [
                        trans_stamped.transform.rotation.x,
                        trans_stamped.transform.rotation.y,
                        trans_stamped.transform.rotation.z,
                        trans_stamped.transform.rotation.w
                    ]
                    r = R.from_quat(rot)      # create rotation object
                    rot_mat = r.as_matrix()
                    found = True
                    self.get_logger().info("Retrieved camera_color_optical_frame -> base_link")
                except Exception as e:
                    self.get_logger().info(f"Unable to retrieve tf between: camera_color_optical_frame -> base_link: {e}")
                    # rate.sleep(0.1)

            self.get_logger().info(YELLOW + f"Transform camera_link -> base_link: {trans}" + END)
            self.get_logger().info(YELLOW + f"Transform camera_link -> base_link: {rot}" + END)

            trans_world_camera = translation_matrix(trans)
            rot_world_camera = quaternion_matrix(rot)
            M_world_camera = np.dot(trans_world_camera, rot_world_camera)

            red_button_world = np.dot(M_world_camera,self.get4Vector(self.red_button_camera))
            blue_button_world = np.dot(M_world_camera,self.get4Vector(self.blue_button_camera))
            screen_world = np.dot(M_world_camera,self.get4Vector(self.screen))
            maze_world = np.dot(M_world_camera,self.get4Vector(self.maze))
            stylus_world = np.dot(M_world_camera,self.get4Vector(self.stylus))
            task_board_world = np.dot(M_world_camera,self.get4Vector(self.task_board))

            

            self.get_logger().info(f"Red button (in base_link) before set z: {red_button_world}")

            def adjust_z(world,z_val):
                vec = world[0:-1]
                vec[-1] = z_val
                return vec

            red_button_world_backup = red_button_world[0:-1]
            red_button_world = adjust_z(red_button_world,0.0)
            blue_button_world = adjust_z(blue_button_world,0.0)
            screen_world = adjust_z(screen_world,0.09)
            maze_world = adjust_z(maze_world,0.085)
            stylus_world = adjust_z(stylus_world,0.105)
            task_board_world = adjust_z(task_board_world,0.09)

            self.get_logger().info(f"Red button (in base_link) after set z: {red_button_world}")
            self.get_logger().info(f"Blue button (in base_link) after set z: {blue_button_world}")

            
            self.get_logger().info(f"BEFOREEEEE Red button (in base_link) after set z: {self.red_button_camera}")
            self.get_logger().info(f"BEFOREEEEE Blue button (in base_link) after set z: {self.blue_button_camera}")
            red_cam = self.red_button_camera.copy()
            blue_cam = self.blue_button_camera.copy()
            
            red_cam[2] = 0
            blue_cam[2] = 0
            
            self.get_logger().info(f"NEWWWWWWWW Red button (in base_link) after set z: {red_cam}")
            self.get_logger().info(f"NEWWWWWWWW Blue button (in base_link) after set z: {blue_cam}")
            self.get_logger().info(f"Red button (in base_link) after set z: {self.red_button_camera}")
            self.get_logger().info(f"Blue button (in base_link) after set z: {self.blue_button_camera}")
            
        
            z_axis = np.array([0.0, 0.0, 1.0])
            x_axis = (blue_cam - red_cam) / np.linalg.norm(blue_cam - red_cam)
            # x_axis = (red_button_world - blue_button_world) / np.linalg.norm(red_button_world - blue_button_world)
            y_axis_first_approach = np.cross(z_axis, x_axis)
            y_axis_norm = y_axis_first_approach / np.linalg.norm(y_axis_first_approach)

            print(f"x {x_axis} y {y_axis_norm}",flush=True)
            fixed_rot_x = np.array([0.9902681,  0.0000000,  -0.1391731])
            fixed_rot_y = np.array([0.0000000,  1.0000000,  0.0000000])
            fixed_rot_z = np.array([0.1391731,  0.0000000,  0.9902681])

            fix_mat = np.array([fixed_rot_x, fixed_rot_y, fixed_rot_z])
            rot_mat_world_board_before = np.array([x_axis, y_axis_norm, z_axis]).T
            # rot_mat_world_board = np.dot(fix_mat,rot_mat_world_board_before)
            rot_mat_world_board = np.dot(rot_mat_world_board_before,fix_mat)

            # rot_mat_world_board = np.array([x_axis, y_axis_norm, z_axis]).T
            M_world_board_only_rot = np.identity(4)
            M_world_board_only_rot[0:-1, 0:-1] = rot_mat_world_board

            M_world_board_only_tra = np.identity(4)
            M_world_board_only_tra[0:3, -1] = np.array([red_button_world_backup[0], red_button_world_backup[1], 0.18])

            M_world_board = np.dot(M_world_board_only_tra, M_world_board_only_rot)
            
            rotation_quat = quaternion_from_matrix(M_world_board)
            print ("rotation quat",rotation_quat)
            
            stylus_rot_x = np.array([0.0,  -1.0,  0.0])
            stylus_rot_y = np.array([1.0,  0.0,  0.0])
            stylus_rot_z = np.array([0.0,  0.0,  1.0])
            stylus_mat = np.array([stylus_rot_x, stylus_rot_y, stylus_rot_z])

            stylus_board = np.dot(rot_mat_world_board,stylus_mat.T)
            stylus_rotation_quat = quaternion_from_matrix(stylus_board)
        

            self.get_logger().info("Publishing tf")

            static_transformStamped_board = TransformStamped()
            static_transform_red = TransformStamped()
            static_transform_blue = TransformStamped()
            static_transform_screen = TransformStamped()
            static_transform_maze = TransformStamped()
            static_transform_stylus = TransformStamped()
            static_transform_task_board = TransformStamped()

            def get_static_transform(parent, child, translation, rotation):
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = parent
                t.child_frame_id = child
                t.transform.translation.x = translation[0]
                t.transform.translation.y = translation[1]
                t.transform.translation.z = translation[2]
                t.transform.rotation.x = float(rotation[0])
                t.transform.rotation.y = float(rotation[1])
                t.transform.rotation.z = float(rotation[2])
                t.transform.rotation.w = float(rotation[3])
                return t
            
            # TODO: Add checks tha frames exist before pub
            frame1 = "camera_color_optical_frame"
            frame2 = "base_link"
            static_transformStamped_board = get_static_transform(frame2, "board", M_world_board[0:3, -1], rotation_quat)
            static_transform_red = get_static_transform(frame1, "red_button", self.red_button_camera, rotation_quat)
            static_transform_blue = get_static_transform(frame1, "blue_button", self.blue_button_camera,rotation_quat)
            if self.screen is not None: static_transform_screen = get_static_transform(frame1, "screen", self.screen, rotation_quat)
            if self.maze is not None: static_transform_maze = get_static_transform(frame1, "maze", self.maze, rotation_quat)
            # if self.stylus is not None: static_transform_stylus = get_static_transform(frame1, "stylus", self.stylus, stylus_rotation_quat)
            if self.task_board is not None: static_transform_task_board = get_static_transform(frame1, "task_board", self.task_board, rotation_quat)



            static_transformStamped_A = TransformStamped()
            static_transformStamped_B = TransformStamped()
            static_transformStamped_Background = TransformStamped()
            static_transformStamped_Up = TransformStamped()
            static_transformStamped_Down = TransformStamped()
            static_transformStamped_Left = TransformStamped()
            static_transformStamped_Right = TransformStamped()
            # stylus_new = TransformStamped()
            
            static_transformStamped_align_frame = TransformStamped()
    
            print("now doing rotation")
            cam_R_to_blue = M_world_board[:3, :3]
            A_position_wrt_blue = np.array([0 , 0.062, 0])
            B_position_wrt_blue = np.array([-0.02 , 0.062, 0])
            Back_position_wrt_blue = np.array([-0.01 , 0.072, 0])
            Up_position_wrt_blue = np.array([-0.01 , 0.082, 0])
            Down_position_wrt_blue = np.array([-0.01 , 0.062, 0])
            Left_position_wrt_blue = np.array([-0.00 , 0.072, 0])
            Right_position_wrt_blue = np.array([-0.02 , 0.072, 0])
            
            stylus_position_wrt_blue = np.array([-0.115 , -0.01, -0.01])
            align_frame_position_wrt_blue = np.array([-0.1 , 0.05, -0.2])      #estimation
            
            A_wrt_cam = np.dot(cam_R_to_blue,A_position_wrt_blue)
            B_wrt_cam = np.dot(cam_R_to_blue,B_position_wrt_blue)
            Back_wrt_cam = np.dot(cam_R_to_blue,Back_position_wrt_blue)
            Up_wrt_cam = np.dot(cam_R_to_blue,Up_position_wrt_blue)
            Down_wrt_cam = np.dot(cam_R_to_blue,Down_position_wrt_blue)
            Left_wrt_cam = np.dot(cam_R_to_blue,Left_position_wrt_blue)
            Right_wrt_cam = np.dot(cam_R_to_blue,Right_position_wrt_blue)
            
            
            stylus_wrt_cam = np.dot(cam_R_to_blue,stylus_position_wrt_blue)
            align_frame_position_wrt_cam = np.dot(cam_R_to_blue,align_frame_position_wrt_blue)
            
            
            A_wrt_cam = A_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            B_wrt_cam = B_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            
            Back_wrt_cam = Back_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            Up_wrt_cam = Up_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            Down_wrt_cam = Down_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            Left_wrt_cam = Left_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            Right_wrt_cam = Right_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            
            
            stylus_wrt_cam = stylus_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            align_frame_position_wrt_cam = align_frame_position_wrt_cam + np.array([self.blue_button_camera[0], self.blue_button_camera[1], self.blue_button_camera[2]])
            
            A_wrt_cam[2] = self.screen[2]
            B_wrt_cam[2] = self.screen[2]
            Back_wrt_cam[2] = self.screen[2]
            Up_wrt_cam[2] = self.screen[2]
            Down_wrt_cam[2] = self.screen[2]
            Left_wrt_cam[2] = self.screen[2]
            Right_wrt_cam[2] = self.screen[2]
            
            
            # B_position = np.array([self.blue_button_camera.x - 0.05 , self.blue_button_camera.y + 0.06, self.screen.z])
            # Background_position = np.array([self.blue_button_camera.x - 0.04 , self.blue_button_camera.y + 0.07, self.screen.z])
            # Up_position = np.array([self.blue_button_camera.x - 0.04 , self.blue_button_camera.y + 0.08, self.screen.z])
            # Down_position = np.array([self.blue_button_camera.x - 0.04 , self.blue_button_camera.y + 0.06, self.screen.z])
            # Left_position = np.array([self.blue_button_camera.x - 0.03 , self.blue_button_camera.y + 0.07, self.screen.z])
            # Right_position = np.array([self.blue_button_camera.x - 0.05 , self.blue_button_camera.y + 0.07, self.screen.z])
            # stylus_new = np.array([self.blue_button_camera.x - 0.01 , self.blue_button_camera.y + 0.07, self.screen.z])
            
            #definition of the orientation matrix of align_frame
            z_axis_align = cam_R_to_blue[:,2]
            y_axis_align = np.array([-1.0, 0.0, 0.0])
            x_axis_align = np.cross(y_axis_align, z_axis_align)
            x_axis_align = x_axis_align / np.linalg.norm(x_axis_align)
            rot_align_frame = np.array([x_axis_align, y_axis_align, z_axis_align]).T
            quat_align_frame =quaternion_from_matrix(rot_align_frame)
            
            
    
            static_transformStamped_A = get_static_transform(frame1, "A", A_wrt_cam, rotation_quat)
            static_transformStamped_B = get_static_transform(frame1, "B", B_wrt_cam, rotation_quat)
            static_transform_stylus = get_static_transform(frame1, "stylus", stylus_wrt_cam, stylus_rotation_quat)
            static_transformStamped_Background = get_static_transform(frame1, "Background", Back_wrt_cam, rotation_quat)
            static_transformStamped_Up = get_static_transform(frame1, "sq_up", Up_wrt_cam, rotation_quat)
            static_transformStamped_Down = get_static_transform(frame1, "sq_down", Down_wrt_cam, rotation_quat)
            static_transformStamped_Left = get_static_transform(frame1, "sq_left", Left_wrt_cam, rotation_quat)
            static_transformStamped_Right = get_static_transform(frame1, "sq_right", Right_wrt_cam, rotation_quat)
            static_transformStamped_align_frame = get_static_transform(frame1, "align_frame", align_frame_position_wrt_cam, quat_align_frame)
        
        
            transforms = [static_transformStamped_board, static_transform_red, static_transform_blue,static_transform_screen, static_transform_maze, static_transform_stylus, static_transform_task_board, static_transformStamped_A, static_transformStamped_B, static_transformStamped_align_frame, static_transformStamped_Background, static_transformStamped_Up, static_transformStamped_Down, static_transformStamped_Left,static_transformStamped_Right]

            for i, t in enumerate(transforms):
                # if not isinstance(t, TransformStamped):
                print(f"Error: Element {i} is not TransformStamped, but {type(t)}")


            self.broadcaster.sendTransform(transforms)

            self.get_logger().info("Published tf")
            # self.online_board_inference.realtime_inference(frame, view_img=True)
            self.publish_status(True)
            response = Trigger.Response()
            self.srv_execution = True
            response.success = True
            response.message = SUCCESSFUL
            return response
        
        except Exception as e:
            self.get_logger().error(f"Task failed: {str(e)}")
            self.srv_execution = False
        # finally:
        #     self.srv_execution = False

    def publish_status(self, status: bool):
        msg = Bool()
        msg.data = status
        self.status_publisher.publish(msg)

        
    def get4Vector(self, vect):
        vet = np.array([0.0, 0.0, 0.0, 1.0])
        vet[:-1] = vect
        return vet

    def cam_deproj(self,x_,y_,depth_frame_):
        reprojection = None
        reprojection =  np.array(
                    self.realsense.deproject(x_, y_,depth_frame_[y_, x_])) / 1000.0
        return reprojection
    # def publish_Tf():
       
def main(args=None):
    rclpy.init(args=args)
    board_loc_node = BoardLocalization()
    try:
        rclpy.spin(board_loc_node)
    except KeyboardInterrupt:
        pass
    finally:
        board_loc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
