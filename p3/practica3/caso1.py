#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros import ActionState
from nav2_msgs.action import NavigateToPose

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = "/scan"            
TOPIC_COLOR = "/color_detected"

ANG_IZQ = 30.0 * math.pi / 180.0
ANG_DER = -ANG_IZQ


green_detected = False


class WanderAndDetect(State):
    def __init__(self, node):
        super().__init__(outcomes={'found_green'})
        self._node = node
        self._pub_cmd = None
        self._sub_scan = None
        self._sub_color = None
        self._last_time_blocked = time.time()
        self._stuck_time = 2.0
        self._last_turn_dir = 1.0

    def execute(self, blackboard: Blackboard) -> str:
        global green_detected
        # Pub/Sub
        self._pub_cmd = self._node.create_publisher(Twist, TOPIC_VEL, 10)
        self._sub_scan = self._node.create_subscription(
            LaserScan, TOPIC_SCAN, self._laser_cb, 10
        )
        self._sub_color = self._node.create_subscription(
            Int32, TOPIC_COLOR, self._color_cb, 10
        )

        # Bucle: procesamos callbacks hasta detectar verde
        while rclpy.ok() and not green_detected:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        # Detenemos robot
        if self._pub_cmd:
            self._pub_cmd.publish(Twist())

        # Limpiamos subscriptions
        if self._sub_scan:
            self._node.destroy_subscription(self._sub_scan)
            self._sub_scan = None
        if self._sub_color:
            self._node.destroy_subscription(self._sub_color)
            self._sub_color = None

        return 'found_green'

    # --- Callbacks ---
    def _laser_cb(self, msg: LaserScan):
        cmd = Twist()
        safe = 3
        fwd = 0.5
        yaw = 0.6

        pos_izq = self._angle_to_index(msg, ANG_IZQ)
        pos_der = self._angle_to_index(msg, ANG_DER)
        d_izq = msg.ranges[pos_izq] if pos_izq is not None else float('inf')
        d_der = msg.ranges[pos_der] if pos_der is not None else float('inf')
        min_dist = min(d_izq, d_der)
        diff = d_izq - d_der
        threshold = 0.3

        if min_dist > safe:
            cmd.linear.x = fwd
            cmd.angular.z = 0.0
            self._last_time_blocked = time.time()
        else:
            cmd.linear.x = 0.05
            if abs(diff) > threshold:
                self._last_turn_dir = -1.0 if diff < 0 else 1.0
            cmd.angular.z = yaw * self._last_turn_dir
            if time.time() - self._last_time_blocked > self._stuck_time:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.2 * self._last_turn_dir
                self._last_time_blocked = time.time()

        if self._pub_cmd:
            self._pub_cmd.publish(cmd)

    def _color_cb(self, msg: Int32):
        global green_detected
        if msg.data > 500:
            green_detected = True
            self._node.get_logger().info(f"Verde detectado")

    @staticmethod
    def _angle_to_index(msg: LaserScan, ang: float) -> int | None:
        if msg.angle_increment == 0.0:
            return None
        idx = int(round((ang - msg.angle_min) / msg.angle_increment))
        if 0 <= idx < len(msg.ranges):
            return idx
        return None


class GotoWPState(ActionState):
    def __init__(self, wp):
        self.wp = wp
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            create_goal_handler=self.create_goal_handler
        )

    def create_goal_handler(self, blackboard):
        pose = Pose()
        pose.position.x = self.wp[0]
        pose.position.y = self.wp[1]
        pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose.pose = pose
        goal.pose.header.frame_id = "map"
        return goal


def main(args=None):
    rclpy.init(args=args)
    node = Node("patrol_yasmin")

    # FSM
    sm = StateMachine(outcomes={'end'})
    wander = WanderAndDetect(node)
    go_home = GotoWPState([-10.0, 10.0])

    sm.add_state('WANDER', wander, transitions={'found_green':'HOME'})
    sm.add_state('HOME', go_home, transitions={'succeeded':'end'})
    sm.set_start_state('WANDER')
    sm.validate()

    outcome = sm.execute(Blackboard())
    node.get_logger().info(f"FSM terminada con outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()