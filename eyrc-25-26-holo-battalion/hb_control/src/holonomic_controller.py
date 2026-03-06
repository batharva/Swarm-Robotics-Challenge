#!/usr/bin/env python3

"""
Improved Multi-Robot Swarm Coordination System
------------------------------------------------
Enhancements:
✔ Stuck detection (robot detects when approach distance not decreasing)
✔ Automatic angle adjustment (robot rotates ±45° and retries approach)
✔ Retry limit for safety
✔ Cleaner robot state machine
✔ Improved readability & maintainability
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from hb_interfaces.msg import BotCmdArray, BotCmd, Poses2D

import numpy as np
import math
import json
import threading
import time

from linkattacher_msgs.srv import AttachLink, DetachLink
from enum import Enum, auto

class CrateColor(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2


class RobotState(Enum):
    IDLE = auto()
    APPROACHING = auto()
    RECOVERY_TURN = auto()      # <--- NEW: rotate to unblock
    ATTACHING = auto()
    LIFTING = auto()
    TRANSPORTING = auto()
    DROPPING = auto()
    RETRACTING = auto()
    RETURNING = auto()
    DOCKED = auto()


class PID:
    def __init__(self, kp, ki, kd, max_out=1.0, min_out=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.min_out = min_out
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error = error

        # clamp
        output = np.clip(output, -self.max_out, self.max_out)

        # deadband
        if abs(output) > 0.01 and abs(output) < self.min_out:
            output = self.min_out if output > 0 else -self.min_out

        return output

    def reset(self):
        self.integral = 0
        self.prev_error = 0
class MultiRobotSwarm(Node):

    def __init__(self):
        super().__init__('multirobot_swarm_controller')

        self.control_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Robot config
        self.ROBOT_CFG = {
            'crystal': {'id': 0, 'dock': (1218.0, 205.0)},
            'glacio': {'id': 4, 'dock': (864.0, 204.0)},
            'frostbite': {'id': 8, 'dock': (1568.0, 202.0)}
        }

        # Drop zones
        self.ZONES = {
            CrateColor.RED:   {'name': "D1", 'cx': 1215, 'cy': 1215},
            CrateColor.GREEN: {'name': "D2", 'cx': 820,  'cy': 2017},
            CrateColor.BLUE:  {'name': "D3", 'cx': 1616, 'cy': 2017}
        }

        # Robot internal dictionaries
        self.robots = {}

        for name, cfg in self.ROBOT_CFG.items():
            self.robots[name] = {
                'id': cfg['id'],
                'name': name,
                'dock': cfg['dock'],
                'pose': {'x': 0, 'y': 0, 'w': 0},
                'state': RobotState.IDLE,
                'attached': False,
                'attachment_in_progress': False,
                'detach_called': False,
                'assigned': None,
                'pose_received': False,

                # PID controllers
                'pid_x': PID(0.3, 0.0005, 0.09, max_out=1500, min_out=0.1),
                'pid_y': PID(0.3, 0.0005, 0.09, max_out=1500, min_out=0.1),
                'pid_w': PID(1.1, 0.001, 0.05, max_out=5000, min_out=0.2),
                'pid_distance': PID(0.6, 0.006, 0.05, max_out=200, min_out=0.3),

                # Inverse kinematics
                'inv_kin': self._calc_inv_kin(),

                # Timing
                't': time.time(),

                # ---------- NEW FIELDS FOR STUCK DETECTION ----------
                'last_dist': None,      # previous distance to target
                'stuck_timer': 0.0,     # how long distance not improving
                'retry_count': 0,       # current attempts
                'max_retries': 3,       # safety limit
                'recovery_angle': 0.0,  # target rotation angle
                'retry_start_w': 0.0    # starting orientation
            }

            # Communication handles
            self.robots[name]['cmd_pub'] = self.create_publisher(
                BotCmdArray, '/bot_cmd', 10)

            self.robots[name]['attach_cli'] = self.create_client(
                AttachLink, '/attach_link', callback_group=self.service_cb_group)

            self.robots[name]['detach_cli'] = self.create_client(
                DetachLink, '/detach_link', callback_group=self.service_cb_group)

        # Global crate system
        self.oneshot_crates = set()
        self.detected_crates = {}
        self.crate_allocation = {}
        self.completed = set()
        self.total_tasks = 4

        # Subscriptions
        self.create_subscription(Poses2D, '/bot_pose',
                                 self.pose_cb, 10, callback_group=self.control_cb_group)
        self.create_subscription(Poses2D, '/crate_pose',
                                 self.crate_cb, 10, callback_group=self.control_cb_group)

        # Main control loop
        self.create_timer(0.03, self.swarm_loop, callback_group=self.control_cb_group)

        self.get_logger().info("===============================================================")
        self.get_logger().info("🤖 Improved Multi-Robot Swarm Controller Initialized")
        self.get_logger().info("===============================================================")
    def _calc_inv_kin(self):
        """Calculate inverse kinematics matrix for a 3-wheel holonomic robot."""
        alpha = [math.radians(a) for a in [30, 150, 270]]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alpha],
            [math.sin(a + math.pi/2) for a in alpha],
            [1, 1, 1]
        ])
        return np.linalg.inv(M)

    def pose_cb(self, msg):
        """Receive robot positions."""
        for pose in msg.poses:
            for r in self.robots.values():
                if pose.id == r['id']:
                    r['pose']['x'] = pose.x
                    r['pose']['y'] = pose.y
                    r['pose']['w'] = math.radians(pose.w)
                    r['pose_received'] = True
                    break

    def crate_cb(self, msg):
        """Detect crates once and store them."""
        for pose in msg.poses:
            crate_id = pose.id
            self.oneshot_crates.add(crate_id)
            color = CrateColor(crate_id % 3)
            self.detected_crates[crate_id] = {
                'x': pose.x,
                'y': pose.y,
                'color': color
            }
    def allocate(self):
        """Round-robin crate assignment to robots."""
        if self.crate_allocation:
            return

        all_crates = sorted(self.oneshot_crates)
        botnames = list(self.robots.keys())

        for idx, crate_id in enumerate(all_crates[:self.total_tasks]):
            bot_name = botnames[idx % len(botnames)]
            if bot_name not in self.crate_allocation:
                self.crate_allocation[bot_name] = []
            self.crate_allocation[bot_name].append(crate_id)

            color_name = CrateColor(crate_id % 3).name
            self.get_logger().info(
                f"🟦 Allocated {color_name} crate {crate_id} to {bot_name}"
            )
    def send_attach_threaded(self, r, crate_id):
        """Send non-blocking attach request with retry logic."""
        if r['attached'] or r['attachment_in_progress']:
            return

        threading.Thread(
            target=self._attach_worker,
            args=(r, crate_id),
            daemon=True
        ).start()
    def _attach_worker(self, r, crate_id):
        r['attachment_in_progress'] = True

        colors = ['blue', 'green', 'red']
        max_attempts = 5
        attempt = 0

        while attempt < max_attempts and not r['attached']:
            for color in colors:
                try:
                    payload = {
                        "model1_name": f"hb_{r['name']}",
                        "link1_name": "arm_link_2",
                        "model2_name": f"crate_{color}_{crate_id}",
                        "link2_name": f"box_link_{crate_id}"
                    }

                    req = AttachLink.Request()
                    req.data = json.dumps(payload)

                    future = r['attach_cli'].call_async(req)

                    start = time.time()
                    while not future.done() and (time.time() - start < 3.0):
                        time.sleep(0.05)

                    if future.done() and future.result() and future.result().success:
                        self.get_logger().info(
                            f"🔗 {r['name']}: Attached crate {crate_id}"
                        )
                        r['attached'] = True
                        r['attachment_in_progress'] = False
                        return

                except Exception:
                    pass

            attempt += 1
            if not r['attached']:
                self.get_logger().warn(
                    f"🔄 {r['name']}: Retry attaching crate {crate_id} "
                    f"({attempt}/{max_attempts})"
                )
                time.sleep(0.4)

        if not r['attached']:
            self.get_logger().error(
                f"❌ {r['name']}: FAILED to attach crate {crate_id}"
            )

        r['attachment_in_progress'] = False
    def send_detach_threaded(self, r, crate_id):
        if r['detach_called']:
            return

        r['detach_called'] = True

        threading.Thread(
            target=self._detach_worker,
            args=(r, crate_id),
            daemon=True
        ).start()
    def _detach_worker(self, r, crate_id):
        self.get_logger().info(
            f"🔓 {r['name']}: Trying to detach crate {crate_id}"
        )

        colors = ['blue', 'green', 'red']

        for color in colors:
            try:
                payload = {
                    "model1_name": f"hb_{r['name']}",
                    "link1_name": "arm_link_2",
                    "model2_name": f"crate_{color}_{crate_id}",
                    "link2_name": f"box_link_{crate_id}"
                }

                req = DetachLink.Request()
                req.data = json.dumps(payload)

                future = r['detach_cli'].call_async(req)

                start = time.time()
                while not future.done() and (time.time() - start < 2.0):
                    time.sleep(0.05)

                if future.done() and future.result() and future.result().success:
                    self.get_logger().info(
                        f"🟫 {r['name']}: Detached crate {crate_id}"
                    )
                    return

            except Exception:
                pass

        self.get_logger().error(
            f"❌ {r['name']}: FAILED to detach crate {crate_id}"
        )
    def publish_cmd(self, r, m1, m2, m3, base=0.0, elbow=0.0):
        # Ensure values are native Python floats (and id is int) so ROS2 message assertions pass
        try:
            m1_f = float(m1)
            m2_f = float(m2)
            m3_f = float(m3)
            base_f = float(base)
            elbow_f = float(elbow)
        except (TypeError, ValueError):
            # Fallback to zeros if conversion fails
            m1_f = 0.0
            m2_f = 0.0
            m3_f = 0.0
            base_f = 0.0
            elbow_f = 0.0

        cmd = BotCmd(id=int(r['id']), m1=m1_f, m2=m2_f, m3=m3_f, base=base_f, elbow=elbow_f)
        msg = BotCmdArray(cmds=[cmd])
        r['cmd_pub'].publish(msg)
    def calc_motion(self, r, target_x, target_y, dt):
        x = r['pose']['x']
        y = r['pose']['y']
        w = r['pose']['w']

        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)

        goal_angle = math.atan2(dy, dx)

        # slow when near target
        if dist > 100:
            v = r['pid_distance'].compute(dist - 30, dt)
        else:
            v = 0
            r['pid_distance'].reset()

        vx = v * math.cos(goal_angle)
        vy = v * math.sin(goal_angle)

        body_vx = vx * math.cos(w) + vy * math.sin(w)
        body_vy = -vx * math.sin(w) + vy * math.cos(w)

        # desired heading
        target_heading = goal_angle - math.pi/3
        angle_err = math.atan2(
            math.sin(target_heading - w),
            math.cos(target_heading - w)
        )

        omega = r['pid_w'].compute(angle_err, dt)

        wheel_vel = r['inv_kin'] @ np.array([[body_vx], [body_vy], [omega]])
        wheel_vel = np.clip(wheel_vel, -250, 250).flatten()

        return wheel_vel.tolist(), dist
    def swarm_loop(self):
        """Main multi-robot coordination loop."""

        # Allocate crates once enough are detected
        if not self.crate_allocation and len(self.detected_crates) >= self.total_tasks:
            self.allocate()

        dt = 0.03
        current_time = time.time()

        # Process each robot
        for bot_name, robot in self.robots.items():

            # If no pose yet → skip
            if not robot['pose_received']:
                continue

            # If robot has no assigned crates → skip
            if bot_name not in self.crate_allocation:
                continue

            # Assign next crate if needed
            if robot['assigned'] is None and self.crate_allocation[bot_name]:
                crate_id = self.crate_allocation[bot_name].pop(0)
                robot['assigned'] = crate_id
                robot['state'] = RobotState.APPROACHING
                robot['t'] = current_time

                # Reset stuck detection
                robot['last_dist'] = None
                robot['retry_count'] = 0
                robot['stuck_timer'] = 0.0

                self.get_logger().info(
                    f"🎯 {bot_name}: Now handling crate {crate_id}"
                )

            crate_id = robot['assigned']
            if crate_id is None:
                continue

            if crate_id not in self.detected_crates:
                continue

            crate = self.detected_crates[crate_id]
            color = crate['color']
            zone = self.ZONES[color]

            # ============================================================
            #                         STATE MACHINE
            # ============================================================

            # ------------------------------------------------------------
            # ☑️ STATE: APPROACHING (now includes STUCK DETECTION)
            # ------------------------------------------------------------
            if robot['state'] == RobotState.APPROACHING:

                wheel_vel, dist = self.calc_motion(robot, crate['x'], crate['y'], dt)
                self.publish_cmd(robot, wheel_vel[0], wheel_vel[1], wheel_vel[2], 0.0, 60.0)

                # --- STUCK DETECTION ---
                if robot['last_dist'] is None:
                    robot['last_dist'] = dist
                else:
                    # If distance is not getting smaller → stuck
                    if dist > robot['last_dist'] - 8:     # did not move at least 8 units closer
                        robot['stuck_timer'] += dt
                    else:
                        robot['stuck_timer'] = 0.0         # reset when moving well

                    robot['last_dist'] = dist

                # If stuck too long, attempt angle recovery
                if robot['stuck_timer'] > 1.3:
                    if robot['retry_count'] < robot['max_retries']:

                        robot['retry_count'] += 1
                        robot['state'] = RobotState.RECOVERY_TURN
                        robot['retry_start_w'] = robot['pose']['w']

                        # Alternate direction: left → right → left → right
                        direction = -1 if robot['retry_count'] % 2 == 0 else 1
                        robot['recovery_angle'] = math.radians(45) * direction

                        self.get_logger().warn(
                            f"⚠️ {bot_name}: Stuck while approaching crate {crate_id}. "
                            f"Attempting recovery turn #{robot['retry_count']}"
                        )
                        continue

                    else:
                        self.get_logger().error(
                            f"❌ {bot_name}: Could not reach crate {crate_id} after retries!"
                        )
                        robot['assigned'] = None
                        robot['state'] = RobotState.RETURNING
                        continue

                # If close enough, attach
                if dist < 225:
                    robot['state'] = RobotState.ATTACHING
                    robot['t'] = current_time

                    # Reset stuck detection
                    robot['stuck_timer'] = 0
                    robot['last_dist'] = None
                    robot['retry_count'] = 0

                    self.get_logger().info(
                        f"✅ {bot_name}: Arrived at crate {crate_id}"
                    )

            # ------------------------------------------------------------
            # 🔄 NEW STATE: RECOVERY_TURN — rotate to get a better angle
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.RECOVERY_TURN:

                target_angle = robot['retry_start_w'] + robot['recovery_angle']
                current_w = robot['pose']['w']

                angle_err = math.atan2(
                    math.sin(target_angle - current_w),
                    math.cos(target_angle - current_w)
                )

                # PID rotation control
                omega = robot['pid_w'].compute(angle_err, dt)
                wheel_vel = robot['inv_kin'] @ np.array([[0],[0],[omega]])
                wheel_vel = np.clip(wheel_vel, -200, 200).flatten().tolist()

                self.publish_cmd(robot, wheel_vel[0], wheel_vel[1], wheel_vel[2], 0.0, 60.0)

                # When robot reaches target rotation
                if abs(angle_err) < math.radians(6):
                    self.get_logger().info(
                        f"🔁 {bot_name}: Recovery rotation complete. Retrying approach."
                    )

                    robot['state'] = RobotState.APPROACHING
                    robot['stuck_timer'] = 0
                    robot['last_dist'] = None
                    continue

            # ------------------------------------------------------------
            # 🔗 STATE: ATTACHING
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.ATTACHING:

                if robot['attached']:
                    self.get_logger().info(f"🔗 {bot_name}: Attachment successful.")
                    robot['state'] = RobotState.LIFTING
                    robot['t'] = current_time
                else:
                    # Start non-blocking attach command
                    if not robot['attachment_in_progress']:
                        self.send_attach_threaded(robot, crate_id)

                self.publish_cmd(robot, 0, 0, 0, 100.0, 60.0)

            # ------------------------------------------------------------
            # 🏋️ STATE: LIFTING
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.LIFTING:

                if current_time - robot['t'] >= 2.0:
                    robot['state'] = RobotState.TRANSPORTING
                    robot['pid_distance'].reset()
                    robot['pid_w'].reset()

                self.publish_cmd(robot, 0, 0, 0, 0.0, 60.0)

            # ------------------------------------------------------------
            # 🚚 STATE: TRANSPORTING crate to drop zone
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.TRANSPORTING:

                wheel_vel, dist_zone = self.calc_motion(robot, zone['cx'], zone['cy'], dt)
                self.publish_cmd(robot, wheel_vel[0], wheel_vel[1], wheel_vel[2], 0.0, 60.0)

                if dist_zone < 150:
                    robot['state'] = RobotState.DROPPING
                    robot['t'] = current_time
                    self.completed.add((bot_name, crate_id))
                    self.get_logger().info(
                        f"📦 {bot_name}: Reached drop zone {zone['name']}"
                    )

            # ------------------------------------------------------------
            # 📤 STATE: DROPPING — bend (40°) then lower (-80°)
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.DROPPING:

                elapsed = current_time - robot['t']

                if elapsed < 1.0:
                    self.publish_cmd(robot, 0, 0, 0, 0.0, 40.0)
                elif elapsed < 2.5:
                    self.publish_cmd(robot, 0, 0, 0, 0.0, 80.0)
                else:
                    self.send_detach_threaded(robot, crate_id)
                    robot['state'] = RobotState.RETRACTING
                    robot['t'] = current_time

            # ------------------------------------------------------------
            # ↩️ STATE: RETRACTING
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.RETRACTING:

                if current_time - robot['t'] >= 1.5:
                    robot['state'] = RobotState.RETURNING
                    robot['attached'] = False
                    robot['detach_called'] = False
                    robot['assigned'] = None

                self.publish_cmd(robot, 0, 0, 0, 0.0, 0.0)

            # ------------------------------------------------------------
            # 🏠 STATE: RETURNING to dock
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.RETURNING:

                dock_x, dock_y = robot['dock']
                wheel_vel, dist_home = self.calc_motion(robot, dock_x, dock_y, dt)

                self.publish_cmd(robot, wheel_vel[0], wheel_vel[1], wheel_vel[2], 0.0, 0.0)

                if dist_home < 100:
                    robot['state'] = RobotState.DOCKED
                    robot['t'] = current_time
                    self.get_logger().info(f"🏠 {bot_name}: Docked successfully.")

            # ------------------------------------------------------------
            # 🛑 STATE: DOCKED
            # ------------------------------------------------------------
            elif robot['state'] == RobotState.DOCKED:
                self.publish_cmd(robot, 0, 0, 0, 0, 0)

        # =============================================================
        # 🎉 MISSION COMPLETE CHECK
        # =============================================================
        if len(self.completed) >= self.total_tasks:
            if all(r['state'] == RobotState.DOCKED for r in self.robots.values()):
                self.get_logger().info("🎉 MISSION COMPLETE — All crates delivered!")
def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSwarm()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()