#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

TOPIC_COLOR = "/color_detected"


class CheckGreenDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckGreen"):
        super().__init__(name=name)
        self.node = None
        self.green_detected = False
        self.sub = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def initialise(self):
        if self.sub is None and self.node:
            self.sub = self.node.create_subscription(
                Int32, TOPIC_COLOR, self._cb, 10
            )

    def _cb(self, msg):
        # Umbral de detección (ajusta según tu sensor)
        if msg.data > 7500:
            self.green_detected = True
            self.node.get_logger().info(f"VERDE DETECTADO")

    def update(self):
        if self.green_detected:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class NavToPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_pose):
        """
        :param name: Nombre del nodo en el árbol
        :param target_pose: Lista [x, y] con las coordenadas
        """
        super().__init__(name=name)
        self.target = target_pose
        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.goal_sent = False
        self.navigation_result = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # Cliente para el servidor de acción de Nav2
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        
        self.goal_sent = False
        self.goal_handle = None
        self.navigation_result = None
        self.node.get_logger().info(f"Iniciando navegación hacia: {self.target}")

    def update(self):
        # 1. Verificar si el cliente de acción está listo
        if not self.action_client.server_is_ready():
            self.node.get_logger().warn("Esperando servidor Nav2...")
            return py_trees.common.Status.RUNNING

        # 2. Enviar la meta si aún no se ha hecho
        if not self.goal_sent:
            self.send_goal()
            return py_trees.common.Status.RUNNING

        # 3. Verificar si ya tenemos un resultado final
        if self.navigation_result == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif self.navigation_result in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            return py_trees.common.Status.FAILURE
        
        # Mientras tanto, seguimos ejecutando
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        
        if new_status == py_trees.common.Status.INVALID and self.goal_handle:
            self.node.get_logger().info(f"Interrumpiendo navegación a {self.name} (Preemption)")
            
            pass

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target[0])
        goal_msg.pose.pose.position.y = float(self.target[1])
        goal_msg.pose.pose.orientation.w = 1.0

        self._future = self.action_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Meta rechazada por Nav2')
            self.navigation_result = GoalStatus.STATUS_ABORTED
            return
        
        self.goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        self.navigation_result = result
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"Llegada exitosa a {self.name}")
        else:
            self.node.get_logger().warn(f"Navegación fallida o cancelada: {result}")


def create_tree():
    
    
    root = py_trees.composites.Sequence(name="PatrolBT", memory=True)
    
    
    search_phase = py_trees.composites.Selector(name="BuscarVerde O IrDestino", memory=False)
    
    check_green = CheckGreenDetected(name="¿EsVerde?")
    
    
    go_to_start = NavToPose(name="Ir a (0,0)", target_pose=[0.0, 0.0])
    
    go_home = NavToPose(name="Volver Casa", target_pose=[-10.0, 10.0])

    # Construcción
    search_phase.add_children([check_green, go_to_start])
    root.add_children([search_phase, go_home])

    return root


def main(args=None):
    rclpy.init(args=args)
    root = create_tree()
    
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
    try:
        tree.setup(timeout=15)
        print("Árbol Nav2 Reactivo Iniciado")
        tree.tick_tock(period_ms=100) # 10 Hz
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()