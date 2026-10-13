#!/usr/bin/env python3
"""
Nodo Yasmin que recorre 4 waypoints usando Nav2 Actions.
- Usa FSM de Yasmin.
- Cada waypoint es un estado que envía un goal a Nav2.
- Cuando llega al último waypoint, vuelve al inicio.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from yasmin.fsm import State, FSM
import math

# Waypoints directamente en el código
WAYPOINTS = [
    {"name": "A", "pose": (2.0, 0.0, 0.0)},
    {"name": "B", "pose": (2.0, 2.0, 1.57)},
    {"name": "C", "pose": (0.0, 2.0, 3.14)},
    {"name": "D", "pose": (0.0, 0.0, -1.57)},
]

class GotoWaypoint(State):
    def __init__(self, fsm, name, pose):
        super().__init__(fsm, name)
        self.pose = pose
        self.nav_client = ActionClient(fsm.node, NavigateToPose, 'navigate_to_pose')

    async def on_enter(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.fsm.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.pose[0]
        goal_msg.pose.pose.position.y = self.pose[1]
        # Convert yaw a quaternion (solo z,w)
        cy = math.cos(self.pose[2]*0.5)
        sy = math.sin(self.pose[2]*0.5)
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.w = cy

        self.fsm.node.get_logger().info(f'Sending goal to {self.name}: {self.pose}')
        self.future = self.nav_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.fsm.node.get_logger().warn(f'Goal {self.name} rejected!')
            self.fsm.transition('next')
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.fsm.node.get_logger().info(f'Arrived at {self.name}')
        self.fsm.transition('next')


class MyFSM(FSM):
    def __init__(self, node):
        super().__init__(node)
        # Crear un estado por waypoint
        for i, wp in enumerate(WAYPOINTS):
            next_state = WAYPOINTS[i+1]['name'] if i+1 < len(WAYPOINTS) else 'ReturnHome'
            self.add_state(GotoWaypoint(self, wp['name'], wp['pose']), next_state)
        # Estado de retorno al inicio
        self.add_state(GotoWaypoint(self, 'ReturnHome', (0.0,0.0,0.0)), 'Finished')
        self.add_state(State(self, 'Finished'), None)


def main(args=None):
    rclpy.init(args=args)
    node = Node('yasmin_nav2_node')
    fsm = MyFSM(node)
    fsm.start('A')  # empieza en el primer waypoint
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
