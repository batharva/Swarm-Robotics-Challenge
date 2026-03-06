#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from hb_interfaces.msg import BotCmdArray, BotCmd, Poses2D
import numpy as np
import math
import json
from linkattacher_msgs.srv import AttachLink, DetachLink
import threading
import time


class PID:
    def __init__(self, kp, ki, kd, max_out=1.0, min_out=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out, self.min_out = max_out, min_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        output = np.clip(output, -self.max_out, self.max_out)
        if abs(output) > 0.01 and abs(output) < self.min_out:
            output = self.min_out if output > 0 else -self.min_out
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class HolonomicNavigationController(Node):
    def __init__(self):
        super().__init__('holonomic_navigation_controller')
        self.control_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.robot_id = 0
        self.crate_id = 0
        self.attached = False
        self.attachment_in_progress = False
        self.detach_called = False
        self.current_pose = {'x': 0.0, 'y': 0.0, 'w': 0.0}
        self.crate_pose = {'x': 0.0, 'y': 0.0}
        self.pose_received = False
        self.crate_received = False
        self.last_time = self.get_clock().now()
        self.state = "APPROACH"
        self.state_start_time = time.time()
        self.min_distance_to_crate = 80.0
        self.stop_distance = 225.0
        self.max_wheel_vel = 250.0
        self.d1_zone = {
            'x_min': 1020, 'x_max': 1410,
            'y_min': 1075, 'y_max': 1355
        }
        self.d1_center = {
            'x': (self.d1_zone['x_min'] + self.d1_zone['x_max']) / 2,
            'y': (self.d1_zone['y_min'] + self.d1_zone['y_max']) / 2
        }
        self.current_base = 0.0
        self.current_elbow = 0.0
        self.target_elbow_lift = 60.0
        self.pid_x = PID(0.35, 0.0005, 0.08, max_out=2500, min_out=0.3)
        self.pid_y = PID(0.35, 0.0005, 0.08, max_out=2500, min_out=0.3)
        self.pid_w = PID(1.5, 0.001, 0.05, max_out=5000, min_out=0.2)
        self.pid_distance = PID(0.6, 0.006, 0.04, max_out=200, min_out=0.5)
        self.create_subscription(Poses2D, '/bot_pose', self.pose_cb, 10, callback_group=self.control_cb_group)
        self.create_subscription(Poses2D, '/crate_pose', self.crate_cb, 10, callback_group=self.control_cb_group)
        self.bot_cmd_publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.attach_cli = self.create_client(AttachLink, '/attach_link', callback_group=self.service_cb_group)
        self.detach_cli = self.create_client(DetachLink, '/detach_link', callback_group=self.service_cb_group)
        alpha = [math.radians(a) for a in [30, 150, 270]]
        M = np.array([[math.cos(a + math.pi/2) for a in alpha],
                      [math.sin(a + math.pi/2) for a in alpha],
                      [1.0, 1.0, 1.0]])
        self.inv_kinematics_matrix = np.linalg.inv(M)
        self.create_timer(0.03, self.control_loop, callback_group=self.control_cb_group)
        self.get_logger().info("HolonomicNavigationController initialized")

    def send_attach_request_threaded(self):
        if self.attached or self.attachment_in_progress:
            return
        thread = threading.Thread(target=self._attach_thread_worker, daemon=True)
        thread.start()

    def _attach_thread_worker(self):
        self.attachment_in_progress = True
        colors = ["blue", "green", "red"]
        max_attempts = 5
        timeout_count = 0
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            timeout_count += 1
            if timeout_count > 10:
                self.get_logger().warn("Timeout waiting for /attach_link service")
                self.attachment_in_progress = False
                return
            self.get_logger().debug('Waiting for /attach_link service...')
        attempt = 0
        while attempt < max_attempts and not self.attached:
            for color in colors:
                try:
                    payload = {
                        "model1_name": "hb_crystal",
                        "link1_name": "arm_link_2",
                        "model2_name": f"crate_{color}_{self.crate_id}",
                        "link2_name": f"box_link_{self.crate_id}"
                    }
                    req = AttachLink.Request()
                    req.data = json.dumps(payload)
                    self.get_logger().info(f"[Attempt {attempt + 1}] Attaching with color {color}...")
                    future = self.attach_cli.call_async(req)
                    timeout = 3.0
                    start_time = self.get_clock().now()
                    while not future.done():
                        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                        if elapsed > timeout:
                            self.get_logger().debug(f"Timeout attaching with color {color}")
                            break
                        time.sleep(0.05)
                    if future.done():
                        response = future.result()
                        if response is not None and response.success:
                            self.get_logger().info(f"✅ Attached with color {color}: {response.message}")
                            self.attached = True
                            self.attachment_in_progress = False
                            return
                except Exception as e:
                    self.get_logger().debug(f"Error attaching with color {color}: {str(e)}")
            attempt += 1
            if not self.attached and attempt < max_attempts:
                self.get_logger().info(f"Retrying attachment (attempt {attempt + 1}/{max_attempts})...")
                time.sleep(0.5)
        if not self.attached:
            self.get_logger().error("❌ Could not attach crate after multiple attempts")
            self.attachment_in_progress = False

    def send_detach_request_threaded(self):
        if self.detach_called:
            return
        self.detach_called = True
        thread = threading.Thread(target=self._detach_thread_worker, daemon=True)
        thread.start()

    def _detach_thread_worker(self):
        self.get_logger().info("🔓 Attempting to detach crate...")
        timeout_count = 0
        while not self.detach_cli.wait_for_service(timeout_sec=1.0):
            timeout_count += 1
            if timeout_count > 10:
                self.get_logger().warn("Timeout waiting for /detach_link service")
                return
            self.get_logger().debug('Waiting for /detach_link service...')
        colors = ["blue", "green", "red"]
        for color in colors:
            try:
                payload = {
                    "model1_name": "hb_crystal",
                    "link1_name": "arm_link_2",
                    "model2_name": f"crate_{color}_{self.crate_id}",
                    "link2_name": f"box_link_{self.crate_id}"
                }
                req = DetachLink.Request()
                req.data = json.dumps(payload)
                self.get_logger().info(f"Detaching crate with color {color}...")
                future = self.detach_cli.call_async(req)
                timeout = 3.0
                start_time = self.get_clock().now()
                while not future.done():
                    elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                    if elapsed > timeout:
                        break
                    time.sleep(0.05)
                if future.done():
                    response = future.result()
                    if response and response.success:
                        self.get_logger().info(f"✅ Detached with color {color}: {response.message}")
                        return
            except Exception as e:
                self.get_logger().debug(f"Error detaching with color {color}: {str(e)}")
        self.get_logger().error("❌ Could not detach crate")

    def pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id == self.robot_id:
                self.current_pose['x'] = pose.x
                self.current_pose['y'] = pose.y
                self.current_pose['w'] = math.radians(pose.w)
                self.pose_received = True

    def crate_cb(self, msg):
        if msg.poses:
            crate = msg.poses[0]
            self.crate_id = crate.id
            self.crate_pose['x'] = crate.x
            self.crate_pose['y'] = crate.y
            self.crate_received = True

    def publish_bot_command(self, m1, m2, m3, base=0.0, elbow=0.0):
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = self.robot_id
        cmd.m1, cmd.m2, cmd.m3 = float(m1), float(m2), float(m3)
        cmd.base = float(base)
        cmd.elbow = float(elbow)
        msg.cmds = [cmd]
        self.bot_cmd_publisher.publish(msg)
        self.current_base = base
        self.current_elbow = elbow

    def calculate_navigation_velocity(self, target_x, target_y, dt):
        x, y, w = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        if distance > 100:
            approach_v = self.pid_distance.compute(distance - 30, dt)
        else:
            approach_v = 0.0
            self.pid_distance.reset()
        goal_vx = approach_v * math.cos(angle_to_target)
        goal_vy = approach_v * math.sin(angle_to_target)
        goal_w = angle_to_target + math.pi / 3
        error_w = math.atan2(math.sin(goal_w - w), math.cos(goal_w - w))
        omega = self.pid_w.compute(error_w, dt)
        vx_body = goal_vx * math.cos(w) + goal_vy * math.sin(w)
        vy_body = -goal_vx * math.sin(w) + goal_vy * math.cos(w)
        wheel_vel = self.inv_kinematics_matrix @ np.array([[vx_body], [vy_body], [omega]])
        wheel_vel = np.clip(wheel_vel, -self.max_wheel_vel, self.max_wheel_vel).flatten()
        return wheel_vel, distance

    def control_loop(self):
        if not self.pose_received or not self.crate_received:
            self.publish_bot_command(0.0, 0.0, 0.0, 0.0, 0.0)
            return
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = self.get_clock().now()
        x, y, w = self.current_pose['x'], self.current_pose['y'], self.current_pose['w']
        crate_x, crate_y = self.crate_pose['x'], self.crate_pose['y']
        distance_to_crate = math.hypot(crate_x - x, crate_y - y)

        if self.state == "APPROACH":
            self.get_logger().debug(f"[APPROACH] Distance: {distance_to_crate:.1f}mm")
            if distance_to_crate <= self.stop_distance:
                self.get_logger().info(f"[APPROACH] Reached crate! Distance: {distance_to_crate:.1f}mm")
                self.state = "ATTACH"
                self.state_start_time = time.time()
                self.publish_bot_command(0.0, 0.0, 0.0, 100.0, 60.0)
                return
            wheel_vel, _ = self.calculate_navigation_velocity(crate_x, crate_y, dt)
            self.publish_bot_command(wheel_vel[0], wheel_vel[1], wheel_vel[2], 0.0, 60.0)

        elif self.state == "ATTACH":
            elapsed = time.time() - self.state_start_time
            if self.attached:
                self.get_logger().info("✅ Attachment successful! Moving to LIFT state...")
                self.state = "LIFT"
                self.state_start_time = time.time()
                return
            if not self.attachment_in_progress:
                self.get_logger().info("Attempting to attach crate...")
                self.send_attach_request_threaded()
            self.publish_bot_command(0.0, 0.0, 0.0, 100.0, 60.0)

        elif self.state == "LIFT":
            elapsed = time.time() - self.state_start_time
            if elapsed >= 2.0:
                self.get_logger().info("Crate lifted! Moving to TRANSPORT state...")
                self.state = "TRANSPORT"
                self.state_start_time = time.time()
                self.pid_distance.reset()
                self.pid_w.reset()
                return
            self.publish_bot_command(0.0, 0.0, 0.0, 0.0, self.target_elbow_lift)

        elif self.state == "TRANSPORT":
            distance_to_d1 = math.hypot(self.d1_center['x'] - x, self.d1_center['y'] - y)
            self.get_logger().debug(f"[TRANSPORT] Distance to D1: {distance_to_d1:.1f}mm")
            if distance_to_d1 <= 150.0:
                self.get_logger().info("✅ Reached D1 zone! Lowering arm to drop crate...")
                self.state = "DROP"
                self.state_start_time = time.time()
                return
            wheel_vel, _ = self.calculate_navigation_velocity(self.d1_center['x'], self.d1_center['y'], dt)
            self.publish_bot_command(wheel_vel[0], wheel_vel[1], wheel_vel[2], 0.0, 0.0)

        elif self.state == "DROP":
            elapsed = time.time() - self.state_start_time
            if elapsed >= 4.0:
                self.get_logger().info("🔓 Arm lowered. Detaching crate...")
                self.send_detach_request_threaded()
                self.state = "RETRACT"
                self.state_start_time = time.time()
                return
            self.publish_bot_command(0.0, 0.0, 0.0, 40.0, 90.0)

        elif self.state == "RETRACT":
            elapsed = time.time() - self.state_start_time
            if elapsed >= 3:
                self.get_logger().info("🎉 Arm retracted. Mission complete!")
                self.state = "DONE"
                return
            self.publish_bot_command(0.0, 0.0, 0.0, 0.0, 0.0)

        elif self.state == "DONE":
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ MISSION COMPLETE - Crate dropped at D1 zone!")
            self.get_logger().info("=" * 60)
            self.publish_bot_command(0.0, 0.0, 0.0, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicNavigationController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
        controller.publish_bot_command(0.0, 0.0, 0.0, 0.0, 0.0)
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()