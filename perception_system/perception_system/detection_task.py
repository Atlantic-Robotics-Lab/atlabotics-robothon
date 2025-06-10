import rclpy
from rclpy.node import Node
# from perception_system.srv import DetectionTask
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
import multiprocessing
from geometry_msgs.msg import Pose, PoseArray

import cv2
import numpy as np
import threading
from skimage.morphology import skeletonize
from .realsense import RealSense

class DetectionNode(Node):
    def __init__(self, realsense_node):
        super().__init__('detection_node')

        self.rs = realsense_node
        self.get_logger().info("Waiting for camera parameters...")
        self.rs.getCameraParam()
        self.rs.waitCameraInfo()
        self.get_logger().info("Camera parameters retrieved correctly")

        self.status_pub = self.create_publisher(String, 'button_status', 10)
        self.result_pub = self.create_publisher(String, 'detection_label', 10)
        self.points_pub = self.create_publisher(PoseArray, 'detection_point', 10)

        self.srv = self.create_service(Trigger, '/taskboard/detect_button', self.detection_callback)
        self.srv = self.create_service(Trigger, '/taskboard/detect_shape', self.shape_callback)
        
        self.get_logger().info("Detection service ready.")


    def shape_callback(self,request,response):

        threading.Thread(target=self.detect_shape, daemon=True).start()
        empty_array = PoseArray()
        empty_array.poses.clear()
        self.points_pub.publish(empty_array)
        response.success = True
        response.message = "Shape detection started"
        return response


        # self.get_logger().info("Running shape detection")
        # result = self.run_shape_detection()
        # self.get_logger().error("Step2 after return")
        # if result:
        #     label, pixels = result
        #     response.success = True
        #     response.message = "Shape detection complete"
        #     # response.label = label
        #     pose_array = PoseArray()
        #     pose_array.header.stamp = self.get_clock().now().to_msg()
        #     pose_array.header.frame_id = "camera"
        #     depth_frame = self.rs.getDistanceFrame()
        #     # Add poses (with position and orientation)
        #     for px, py in pixels:
        #         pose = Pose()
        #         print(f"pixel {px} and {py}")
        #         # pose.position.x, pose.position.y, pose.position.z = float(px), float(py), 0.0
        #         feature_camera = np.array(self.rs.deproject(px, py, depth_frame[py, px])) / 1000.0
        #         pose.position.x, pose.position.y, pose.position.z = feature_camera[0], feature_camera[1], feature_camera[2]
        #         # response.points.append(pt)
        #         pose_array.poses.append(pose)
        #     self.result_pub.publish(String(data=label))
        #     if pixels:
        #         # self.points_pub.publish(response.points[0])
        #         self.points_pub.publish(pose_array)
        # else:
        #     response.success = False
        #     response.message = "Shape detection failed"            
        # return response
            
    def detect(self):
        self.get_logger().info("Button detection loop started.")
        while self.identify:
            self.get_logger().info("Button detection loop started.")
            self.rs.acquireOnceBoth()
            frame = self.rs.getColorFrame()
            if frame is None:
                self.get_logger().error("No frame received.")
                return

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define HSV ranges
            lower_blue_glow = np.array([87, 250, 240])
            upper_blue_glow = np.array([92, 255, 255])
            lower_blue_idle = np.array([97, 245, 190])
            upper_blue_idle = np.array([105, 255, 230])

            lower_red_glow1 = np.array([5, 170, 240])
            upper_red_glow1 = np.array([17, 220, 255])
            lower_red_glow2 = np.array([170, 200, 200])
            upper_red_glow2 = np.array([179, 255, 255])

            lower_red_idle1 = np.array([2, 245, 155])
            upper_red_idle1 = np.array([5, 255, 175])
            lower_red_idle2 = np.array([170, 150, 70])
            upper_red_idle2 = np.array([179, 210, 130])

            # Create masks
            mask_blue_glow = cv2.inRange(hsv, lower_blue_glow, upper_blue_glow)
            mask_blue_idle = cv2.inRange(hsv, lower_blue_idle, upper_blue_idle)
            mask_red_glow = cv2.inRange(hsv, lower_red_glow1, upper_red_glow1) | cv2.inRange(hsv, lower_red_glow2, upper_red_glow2)
            mask_red_idle = cv2.inRange(hsv, lower_red_idle1, upper_red_idle1) | cv2.inRange(hsv, lower_red_idle2, upper_red_idle2)

            def get_largest_contour_center(mask):
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest)
                    center = (x + w // 2, y + h // 2)
                    return (x, y, w, h), center
                return None, None

            box_rg, center_rg = get_largest_contour_center(mask_red_glow)
            box_ri, center_ri = get_largest_contour_center(mask_red_idle)
            box_bg, center_bg = get_largest_contour_center(mask_blue_glow)
            box_bi, center_bi = get_largest_contour_center(mask_blue_idle)

            result_img = frame.copy()
            target = None
            target_center = None
            BOX_WIDTH, BOX_HEIGHT = 60, 60

            if center_rg and center_bi:
                target = "Red"
                cx, cy = center_rg
                x = cx - BOX_WIDTH // 2
                y = cy - BOX_HEIGHT // 2
                cv2.rectangle(result_img, (x, y), (x + BOX_WIDTH, y + BOX_HEIGHT), (0, 0, 255), 2)
                cv2.circle(result_img, (cx, cy), 5, (0, 0, 255), -1)
                target_center = (cx, cy)

            elif center_bg and center_ri:
                target = "Blue"
                cx, cy = center_bg
                x = cx - BOX_WIDTH // 2
                y = cy - BOX_HEIGHT // 2
                cv2.rectangle(result_img, (x, y), (x + BOX_WIDTH, y + BOX_HEIGHT), (255, 0, 0), 2)
                cv2.circle(result_img, (cx, cy), 5, (255, 0, 0), -1)
                target_center = (cx, cy)

            if target:
                self.get_logger().info(f"{target} Glowing detected at pixel {target_center}")
                self.result_pub.publish(String(data=target))
                
                pixel_msg = Point()
                pixel_msg.x = float(target_center[0])
                pixel_msg.y = float(target_center[1])
                pixel_msg.z = 0.0
                # self.points_pub.publish(pixel_msg)
                
                # self.publish_status(True)
                self.publish_status(target)
                self.identify = False
                self.identify = False
                def plot():
                    plt.figure(multiprocessing.current_process().name)
                    plt.imshow(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB))
                    plt.title("Button Detector")
                    plt.axis('off')
                    plt.show()
                p = multiprocessing.Process(target=plot)
                p.start()
                    
                return
            else:
                self.get_logger().info("No glowing button detected.")

    def detect_shape(self):
        self.get_logger().info("Running shape detection")
        result = self.run_shape_detection()
        if result:
            self.get_logger().info("Detetion done")
            label, pixels = result
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "camera_color_optical_frame"

            print(pixels,flush=True)
            for px, py in pixels:
                pose = Pose()
                depth_value = 100.0
                print(f"label {label} and pixel {px} and {py} with depth {depth_value}",flush=True)
                # feature_camera = np.array(self.rs.deproject(px, py, depth_value)) / 1000.0
                feature_camera = np.array(self.rs.deproject_pixel_to_known_height(px, py, depth_value)) / 1000.0
                print(f"feature_camera {feature_camera}",flush=True)
                pose.position.x, pose.position.y, pose.position.z = feature_camera[0], feature_camera[1], feature_camera[2]
                pose_array.poses.append(pose)
            self.result_pub.publish(String(data=label))
            if pixels:
                self.points_pub.publish(pose_array)
        else:
            self.get_logger().warn("No shape detected.")
            pose = Pose()
            pose_array = PoseArray()
            feature_camera = np.array(self.rs.deproject_pixel_to_known_height(0, 0, 100.0)) / 1000.0
            print(f"feature_camera {feature_camera}",flush=True)
            pose.position.x, pose.position.y, pose.position.z = feature_camera[0], feature_camera[1], feature_camera[2]
            pose_array.poses.append(pose)
            pose_array.poses.clear()
            self.points_pub.publish(pose_array)
            self.result_pub.publish(String(data="No shape detected"))

    def detection_callback(self, request, response):
        self.identify = True
        threading.Thread(target=self.detect, daemon=True).start()
        # self.publish_status(False)
        self.publish_status("None")
        response.success = True
        response.message = "Button detection started"
        return response

    def publish_status(self, status: String):
        # msg = Bool()
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def run_button_detection(self):
        self.rs.acquireOnceBoth()
        frame = self.rs.getColorFrame()
        if frame is None:
            self.get_logger().error("No frame received.")
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_blue_glow = np.array([87, 250, 240])
        upper_blue_glow = np.array([92, 255, 255])
        lower_blue_idle = np.array([97, 245, 190])
        upper_blue_idle = np.array([105, 255, 230])

        lower_red_glow1 = np.array([5, 170, 240])
        upper_red_glow1 = np.array([17, 220, 255])
        lower_red_glow2 = np.array([170, 200, 200])
        upper_red_glow2 = np.array([179, 255, 255])

        lower_red_idle1 = np.array([2, 245, 155])
        upper_red_idle1 = np.array([5, 255, 175])
        lower_red_idle2 = np.array([170, 150, 70])
        upper_red_idle2 = np.array([179, 210, 130])

        def get_largest_contour_center(mask):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                center = (x + w // 2, y + h // 2)
                return center
            return None

        mask_blue_glow = cv2.inRange(hsv, lower_blue_glow, upper_blue_glow)
        mask_blue_idle = cv2.inRange(hsv, lower_blue_idle, upper_blue_idle)
        mask_red_glow = cv2.inRange(hsv, lower_red_glow1, upper_red_glow1) | cv2.inRange(hsv, lower_red_glow2, upper_red_glow2)
        mask_red_idle = cv2.inRange(hsv, lower_red_idle1, upper_red_idle1) | cv2.inRange(hsv, lower_red_idle2, upper_red_idle2)

        center_rg = get_largest_contour_center(mask_red_glow)
        center_ri = get_largest_contour_center(mask_red_idle)
        center_bg = get_largest_contour_center(mask_blue_glow)
        center_bi = get_largest_contour_center(mask_blue_idle)

        if center_rg and center_bi:
            return "Red Glowing", center_rg
        elif center_bg and center_ri:
            return "Blue Glowing", center_bg
        else:
            self.get_logger().info("No glowing button detected.")
            return None

    def run_shape_detection(self):
        self.rs.acquireOnceBoth()
        frame = self.rs.getColorFrame()
        if frame is None:
            self.get_logger().error("No frame received.")
            return None

        image = frame
        self.get_logger().error("Step1")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, white_mask_loose = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(white_mask_loose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        screen_contour = max(contours, key=cv2.contourArea)
        screen_mask = np.zeros_like(gray)
        cv2.drawContours(screen_mask, [screen_contour], -1, 255, -1)
        masked_gray = cv2.bitwise_and(gray, gray, mask=screen_mask)
        blurred = cv2.GaussianBlur(masked_gray, (9, 9), 2)
        detected_circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=12, param1=50, param2=18, minRadius=5, maxRadius=25)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_dot = np.array([12, 150, 150])
        upper_dot = np.array([22, 255, 255])
        cleaned_image = image.copy()
        if detected_circles is not None:
            detected_circles = np.uint16(np.around(detected_circles))
            min_keep_radius = 22
            for circle in detected_circles[0, :]:
                x, y, r = circle
                x1, y1 = max(x - r, 0), max(y - r, 0)
                x2, y2 = min(x + r, image.shape[1]), min(y + r, image.shape[0])
                roi_hsv = hsv[y1:y2, x1:x2]
                # mask_orange = cv2.inRange(roi_hsv, lower_dot, upper_dot)
                # orange_ratio = np.sum(mask_orange > 0) / (roi_hsv.shape[0] * roi_hsv.shape[1])
                if( r < min_keep_radius): # and orange_ratio < 0.2:
                    cv2.circle(cleaned_image, (x, y), int(r * 1.35), (0, 0, 0), -1)
                    
        masked_cleaned = cv2.bitwise_and(cleaned_image, cleaned_image, mask=screen_mask)
        hsv_cleaned = cv2.cvtColor(masked_cleaned, cv2.COLOR_BGR2HSV)

        # lower_red_orange = np.array([1, 140, 180])
        # upper_red_orange = np.array([7, 178, 230])
        # red_mask = cv2.inRange(hsv_cleaned, lower_red_orange, upper_red_orange)
        lower_red1 = np.array([0, 80, 150])
        upper_red1 = np.array([12, 250, 255])
        
        # Upper red range (wraps around hue circle)
        lower_red2 = np.array([155, 80, 150])
        upper_red2 = np.array([179, 200, 255])
        
        # Combine both masks
        mask1 = cv2.inRange(hsv_cleaned, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_cleaned, lower_red2, upper_red2)
        
        red_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        main_contour = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(red_mask)
        cv2.drawContours(mask, [main_contour], -1, 255, 2)
        binary_normalized = (mask > 0).astype(np.uint8)
        skeleton = skeletonize(binary_normalized).astype(np.uint8) * 255
        ys, xs = np.where(skeleton == 255)
        points = np.array(list(zip(xs, ys)), dtype=np.int32)
        shape_type = ""
        shape_pixels = []
        if len(points) >= 5:
            ellipse = cv2.fitEllipse(points)
            (xc, yc), (major, minor), angle = ellipse
            circularity_score = abs(major - minor) / max(major, minor)
            ellipse_area = np.pi * (major / 2) * (minor / 2)
            rect = cv2.minAreaRect(points)
            box = cv2.boxPoints(rect).astype(int)
            rect_area = cv2.contourArea(box)
            # rect_fit_ratio = abs(rect_area - ellipse_area) / max(rect_area, ellipse_area)
            if circularity_score < 0.25:
                shape_type = "Circle"
                center = (int(xc), int(yc))
                a, b = int(major / 2), int(minor / 2)
                angle_rad = np.deg2rad(angle)
                for t in np.linspace(0, 2 * np.pi, 8, endpoint=False):
                    x = int(center[0] + a * np.cos(t) * np.cos(angle_rad) - b * np.sin(t) * np.sin(angle_rad))
                    y = int(center[1] + a * np.cos(t) * np.sin(angle_rad) + b * np.sin(t) * np.cos(angle_rad))
                    shape_pixels.append((x, y))
        if not shape_pixels:
            hull = cv2.convexHull(points)
            epsilon = 0.04 * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)
            if len(approx) == 3:
                shape_type = "Triangle"
                shape_pixels = [tuple(pt[0]) for pt in approx]
            else:
                rect = cv2.minAreaRect(points)
                box = cv2.boxPoints(rect).astype(int)
                def order_box_points(pts):
                    pts = sorted(pts, key=lambda x: x[0])
                    left = sorted(pts[:2], key=lambda x: x[1])
                    right = sorted(pts[2:], key=lambda x: x[1])
                    return [left[0], right[0], right[1], left[1]]
                shape_pixels = order_box_points(box)
                shape_type = "Square"
        if shape_pixels:
            if not np.array_equal(shape_pixels[0], shape_pixels[-1]):
                shape_pixels.append(shape_pixels[0])
        else:
            return None
        # fig, ax = plt.subplots()
        # ax.imshow(cv2.cvtColor(red_mask, cv2.COLOR_BGR2RGB))
        # xs, ys = zip(*shape_pixels)
        # ax.scatter(xs, ys, c='lime', s=70, label='Detected Shape')
        # ax.set_title("Shape Detection Result")
        # ax.legend()
        # plt.show()

        return shape_type, shape_pixels

def main(args=None):
    rclpy.init(args=args)
    rs = RealSense()
    node = DetectionNode(rs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# ros2 service call /robothon2023/detection_service perception_system/srv/DetectionTask "{mode: 1}"
