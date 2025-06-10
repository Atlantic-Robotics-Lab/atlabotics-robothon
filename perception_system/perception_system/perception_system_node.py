
from .board_localization import BoardLocalization
from .realsense import RealSense
from .detection_task import DetectionNode
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    try:
        rs_cam = RealSense()
        board_loc = BoardLocalization(rs_cam)
        detection = DetectionNode(rs_cam)

        executor = MultiThreadedExecutor()
        executor.add_node(rs_cam)
        executor.add_node(board_loc)
        executor.add_node(detection)
        try:
            # Execute callbacks for both nodes as they become ready
            executor.spin()
        finally:
            executor.shutdown()
            board_loc.destroy_node()
            rs_cam.destroy_node()
            detection.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
