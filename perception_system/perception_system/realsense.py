#! /usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from message_filters import Subscriber, TimeSynchronizer

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs2

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'

# Realsense Topic
COLOR_FRAME_TOPIC = '/camera/camera/color/image_raw'
DEPTH_ALIGNED_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'

CAMERA_FRAME = "camera_color_optical_frame"

PARAMETERS_LOG = 'Camera Parameters acquired \n  Parameters:{}'

class RealSense(Node):
    def __init__(self):
        super().__init__('realsense_node')

        self.bridge = CvBridge()
        self.colorFrame = None
        self.depthFrame = None
        self.frameRgb = None
        self.frameDepth = None

        self.intrinsics = None
        self.cameraInfoReceived = False
        self.frameAcquired = False
        self.frame_number = 0

        self.create_subscription(Image, COLOR_FRAME_TOPIC, self.color_callback, 10)
        self.create_subscription(Image, DEPTH_ALIGNED_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.cameraInfoCallback, 10)

    
    def color_callback(self, msg: Image):
        # Convert and store the color frame
        self.frameRgb = msg
        # self.get_logger().info(YELLOW + f'Received color image at timestamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}' + END)

    def depth_callback(self, msg: Image):
        # Convert and store the depth frame
        self.frameDepth = msg
        # self.get_logger().info(f'Received depth image at timestamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

    def callback(self, frameRgb, frameDepth):
        colorFrame = self.bridge.imgmsg_to_cv2(frameRgb, desired_encoding="passthrough")
        depthFrame = self.bridge.imgmsg_to_cv2(frameDepth, desired_encoding="passthrough")
        frameDistance = self.bridge.imgmsg_to_cv2(frameDepth, desired_encoding="32FC1")

        self.colorFrame = colorFrame.copy()
        self.depthFrame = depthFrame.copy()
        self.frameDistance = frameDistance.copy()

    # def callbackOnlyRgb(self, frameRgb):
    #     colorFrame = self.bridge.imgmsg_to_cv2(frameRgb, desired_encoding="passthrough")
    #     self.colorFrame = colorFrame.copy()

    def cameraInfoCallback(self, cameraInfo):
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.k[2]
        self.intrinsics.ppy = cameraInfo.k[5]
        self.intrinsics.fx = cameraInfo.k[0]
        self.intrinsics.fy = cameraInfo.k[4]

        if cameraInfo.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.d]
        self.cameraInfoReceived = True

        self.frame_id = cameraInfo.header.frame_id
        # self.get_logger().info("Camera frame id: {}".format(self.frame_id))

    def waitCameraInfo(self):
        while rclpy.ok() and not self.cameraInfoReceived:
            rclpy.spin_once(self)
        # self.camera_info_sub.destroy()
        self.get_logger().info(PARAMETERS_LOG.format(self.intrinsics))

    def acquire(self):
        self.color_sub = Subscriber(self, Image, COLOR_FRAME_TOPIC)
        self.depth_sub = Subscriber(self, Image, DEPTH_ALIGNED_TOPIC)
        ts = TimeSynchronizer([self.color_sub, self.depth_sub], 10)
        ts.registerCallback(self.callback)
        # rclpy.spin(self)

    def acquireOnlyRgb(self):
        # self.color_sub = self.create_subscription(Image, COLOR_FRAME_TOPIC, self.callbackOnlyRgb, 10)
        colorFrame = self.bridge.imgmsg_to_cv2(self.frameRgb, desired_encoding="passthrough")
        self.colorFrame = colorFrame.copy()

    def acquireOnce(self):
        self.get_logger().info("Waiting frame ...")
        # self.create_subscription(Image, COLOR_FRAME_TOPIC, self.color_callback, 10)
        colorFrame = self.bridge.imgmsg_to_cv2(self.frameRgb, desired_encoding="bgr8")
        self.colorFrame = colorFrame.copy()
        self.get_logger().info("Frame received...")

    def acquireOnceBoth(self):
        self.get_logger().info("Waiting frame ...")
        # self.create_subscription(Image, COLOR_FRAME_TOPIC, self.color_callback, 10)
        # self.create_subscription(Image, DEPTH_ALIGNED_TOPIC, self.depth_callback, 10)

        if self.frameRgb is not None:
            self.get_logger().warn("READING")
            colorFrame = self.bridge.imgmsg_to_cv2(self.frameRgb, desired_encoding="bgr8")
            self.colorFrame = colorFrame.copy()
        else:
            self.get_logger().warn("Color frame not received yet.")
        
        if self.frameDepth is not None:
            depthFrame = self.bridge.imgmsg_to_cv2(self.frameDepth, desired_encoding="passthrough")
            frameDistance = self.bridge.imgmsg_to_cv2(self.frameDepth, desired_encoding="32FC1")

            self.depthFrame = depthFrame.copy()
            self.frameDistance = frameDistance.copy()
        else:
            self.get_logger().warn("Depth frame not received yet.")
    
    def getColorFrame(self):
        return self.colorFrame

    def getDistanceFrame(self):
        # align = rs2.align(rs2.stream.color)
        # aligned_frames = align.process(self.frameRgb)
        # depth_frame = aligned_frames.get_depth_frame()
        return self.frameDistance
        # return depth_frame

    def saveImage(self, folder_path):
        self.acquireOnce()
        cv2.imwrite(folder_path + "frame_" + str(self.frame_number) + ".png", self.colorFrame)
        self.frame_number += 1

    def saveAquiredImage(self, full_name):
        if self.colorFrame is not None:
            cv2.imwrite(full_name, self.colorFrame)

    def showColorFrame(self, nameWindowRgb):
        imgImshow = cv2.cvtColor(self.colorFrame, cv2.COLOR_RGB2BGR)
        cv2.imshow(nameWindowRgb, imgImshow)

    def showImage(self, nameWindowRgb, nameWindowDepth):
        imgImshow = cv2.cvtColor(self.colorFrame, cv2.COLOR_RGB2BGR)
        cv2.imshow(nameWindowRgb, imgImshow)
        cv2.imshow(nameWindowDepth, self.depthFrame)

    def getCameraParam(self):
        # self.camera_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.cameraInfoCallback, 10)
        self.intrinsics = self.intrinsics

    def stop(self):
        self.color_sub.destroy()
        self.depth_sub.destroy()

    def deproject(self, x, y, depth):
        print(self.intrinsics)
        print([x,y])
        print(depth)
        deprojection = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)
        print(deprojection)
        # self.debug_deprojection(self.intrinsics,self.depthFrame,self.colorFrame,x,y )
        # return (deprojection[2], -deprojection[0], -deprojection[1])
        return deprojection


    def deproject_pixel_to_known_height(self,u, v, known_z):
        """
        Deprojects a 2D pixel (u, v) onto a 3D point on a plane at known height Z in the camera frame.

        Args:
            u, v: Pixel coordinates
            intrinsics: rs.intrinsics (from profile.get_stream().as_video_stream_profile().get_intrinsics())
            known_z: The known Z height (meters) of the plane in camera frame

        Returns:
            A 3D point [X, Y, Z] in camera frame
        """
        # Compute normalized direction vector
        x = (u - self.intrinsics.ppx) / self.intrinsics.fx
        y = (v - self.intrinsics.ppy) / self.intrinsics.fy
        z = 1.0
        ray = np.array([x, y, z])
        ray = ray / np.linalg.norm(ray)

        # Scale ray to reach the known Z height
        scale = known_z / ray[2]  # since ray.z = 1.0 before normalization
        point_3d = ray * scale
        return point_3d

# # 
# def main(args=None):
#     rclpy.init(args=args)
  

#     try:
#         node = RealSense()
#         node.getCameraParam()
#         node.waitCameraInfo()
#         node.acquire()
#         rclpy.spin(node)  # Only if RealSense inherits from Node (otherwise just keep it running some other way)
#     except KeyboardInterrupt:
#         print("Shutting down safely...")
#     finally:
#     # On shutdown
#         node.stop()
#         rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RealSense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.get_logger().info("HERE?????????????")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()