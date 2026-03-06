#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from hb_interfaces.msg import Pose2D, Poses2D


class PoseDetector(Node):
    """
    Detects ArUco markers and publishes robot & crate poses
    using homography-based pixel → world mapping.
    """

    def __init__(self):
        super().__init__('localization_node')

        self.bridge = CvBridge()

        # ================= PARAMETERS =================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params
        )

        self.bot_marker_ids = [0, 2, 4, 8]

        # World size in mm
        self.corner_world_coords = {
            1: [0, 0],
            3: [2438.4, 0],
            7: [2438.4, 2438.4],
            5: [0, 2438.4]
        }

        # ================= CAMERA =================
        self.camera_matrix = None
        self.dist_coeffs = None

        # ================= HOMOGRAPHY =================
        self.H = None
        self.homography_ready = False

        # ================= SMOOTHING =================
        self.alpha = 0.4   # smoothing factor
        self.prev_poses = {}

        # ================= ROS =================
        self.create_subscription(
            Image, "/camera/image_raw",
            self.image_callback, 10
        )
        self.create_subscription(
            CameraInfo, "/camera/camera_info",
            self.camera_info_callback, 10
        )

        self.crate_pub = self.create_publisher(
            Poses2D, "/crate_pose", 10
        )
        self.bot_pub = self.create_publisher(
            Poses2D, "/bot_pose", 10
        )

        self.get_logger().info("✅ PoseDetector initialized")


    # ================= CAMERA INFO =================
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics loaded")


    # ================= IMAGE CALLBACK =================
    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )

            image = cv2.undistort(
                image, self.camera_matrix, self.dist_coeffs
            )
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = self.detector.detectMarkers(gray)
            if ids is None:
                cv2.imshow("ArUco", image)
                cv2.waitKey(1)
                return

            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            # ================= HOMOGRAPHY =================
            if not self.homography_ready:
                self.try_compute_homography(corners, ids)

            if not self.homography_ready:
                cv2.imshow("ArUco", image)
                cv2.waitKey(1)
                return

            bot_poses = {}
            crate_poses = {}

            for i, marker_id in enumerate(ids):

                if marker_id in self.corner_world_coords:
                    continue

                marker = corners[i][0]
                cx, cy = marker.mean(axis=0)

                wx, wy = self.pixel_to_world(cx, cy)

                # Orientation: top edge direction
                edge = marker[1] - marker[0]
                yaw = math.atan2(edge[1], edge[0])

                pose = self.smooth_pose(
                    marker_id, wx, wy, yaw
                )

                if marker_id in self.bot_marker_ids:
                    bot_poses[marker_id] = pose
                else:
                    crate_poses[marker_id] = pose

                cv2.putText(
                    image,
                    f"({wx/1000:.2f},{wy/1000:.2f},{yaw:.2f})",
                    (int(cx) - 30, int(cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (0, 255, 0), 1
                )

            if bot_poses:
                self.publish_poses(self.bot_pub, bot_poses)
            if crate_poses:
                self.publish_poses(self.crate_pub, crate_poses)

            cv2.imshow("ArUco", image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(str(e))


    # ================= HOMOGRAPHY =================
    def try_compute_homography(self, corners, ids):

        pixel_pts = []
        world_pts = []

        corner_map = {
            1: 0,
            3: 1,
            7: 2,
            5: 3
        }

        for i, marker_id in enumerate(ids):
            if marker_id in corner_map:
                idx = corner_map[marker_id]
                pixel_pts.append(corners[i][0][idx])
                world_pts.append(self.corner_world_coords[marker_id])

        if len(pixel_pts) == 4:
            self.H, _ = cv2.findHomography(
                np.array(pixel_pts),
                np.array(world_pts)
            )
            self.homography_ready = True
            self.get_logger().info("✅ Homography computed")


    # ================= UTILITIES =================
    def pixel_to_world(self, x, y):
        pt = np.array([[[x, y]]], dtype=np.float32)
        world = cv2.perspectiveTransform(pt, self.H)
        return float(world[0][0][0]), float(world[0][0][1])

    def smooth_pose(self, marker_id, x, y, w):
        if marker_id not in self.prev_poses:
            self.prev_poses[marker_id] = (x, y, w)
            return (x, y, w)

        px, py, pw = self.prev_poses[marker_id]
        x = self.alpha * x + (1 - self.alpha) * px
        y = self.alpha * y + (1 - self.alpha) * py
        w = self.alpha * w + (1 - self.alpha) * pw

        self.prev_poses[marker_id] = (x, y, w)
        return (x, y, w)

    def publish_poses(self, publisher, poses):
        msg = Poses2D()
        for mid, (x, y, w) in poses.items():
            p = Pose2D()
            p.id = int(mid)
            p.x = x
            p.y = y
            p.w = w
            msg.poses.append(p)
        publisher.publish(msg)


# ================= MAIN =================
def main():
    rclpy.init()
    node = PoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()