#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32

from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros import ActionState



green_detected = False


class GotoWPState(ActionState):
    def __init__(self, wp, name="WP", show_distance=True, yaw=None):
        self.wp = wp
        self.name = name
        self.show_distance = show_distance
        self.yaw = yaw  # Orientación en radianes (None = sin orientación específica)
        self.last_print_time = 0
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            create_goal_handler=self.create_goal_handler,
            feedback_handler=self.feedback_handler,
            result_handler=self.result_handler,
        )

    def create_goal_handler(self, blackboard):
        import math
        pose = Pose()
        pose.position.x = self.wp[0]
        pose.position.y = self.wp[1]
        
        # Si NO hay yaw específico, mantener orientación actual (no reorientar)
        if self.yaw is None:
            pose.orientation.w = 1.0
            pose.orientation.z = 0.0
        else:
            # Convertir yaw (radianes) a quaternion
            pose.orientation.z = math.sin(self.yaw / 2.0)
            pose.orientation.w = math.cos(self.yaw / 2.0)
        
        goal = NavigateToPose.Goal()
        goal.pose.pose = pose
        goal.pose.header.frame_id = "map"
        
        if self.name == "HOME":
            print(" Volviendo a HOME...")
        else:
            yaw_info = f", yaw={math.degrees(self.yaw):.0f}°" if self.yaw is not None else ""
            print(f" [{self.name}] Navegando a ({self.wp[0]}, {self.wp[1]}{yaw_info})")
        
        self.last_print_time = 0  # Reset del timer
        return goal

    def feedback_handler(self, blackboard, fb):
        if not self.show_distance:
            return
        
        import time
        current_time = time.time()
        
        # Solo imprime cada 1 segundo
        if current_time - self.last_print_time >= 1.0:
            print(f"[{self.name}] Distancia restante: {round(fb.distance_remaining, 2)} m")
            self.last_print_time = current_time
    
    def result_handler(self, blackboard, result):
        if self.name != "HOME":
            print(f"[{self.name}] Waypoint alcanzado")
        return "succeeded"



class CheckGreenState(State):
    def __init__(self):
        super().__init__(outcomes=["found", "not_found"])

    def execute(self, blackboard):
        global green_detected
        if green_detected:
            print("¡Verde detectado!")
            return "found"
        return "not_found"



class GreenListener(Node):
    def __init__(self):
        super().__init__("green_listener")
        self.sub = self.create_subscription(
            Int32, "/color_detected", self.callback, 10
        )
        print("[VISION] Listener iniciado… esperando verde…")

    def callback(self, msg):
        global green_detected
        if msg.data > 500:  # Umbral cambiado a 500
            green_detected = True
            print(f"Verde detectado! ({msg.data} píxeles)")


def main(args=None):
    rclpy.init(args=args)

    # Nodo ROS2 para el listener
    listener = GreenListener()

    # Crear executor en hilo separado para procesar callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(listener)
    
    # Ejecutar spin en un thread separado
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Blackboard global para Yasmin
    bb = Blackboard()

    
    import math

    
    
    wp1 = GotoWPState([-11.0, -13.0], "WP1", show_distance=True, yaw=None)
    wp2 = GotoWPState([13.0, -10.0], "WP2", show_distance=True, yaw=None)
    wp3 = GotoWPState([8.0, 0.0], "WP3", show_distance=True, yaw=None)

    
    home = GotoWPState([-10.0, 10.0], "HOME", show_distance=False, yaw=0)

    # Estado check
    check = CheckGreenState()

    # Estado especial que decide el siguiente waypoint
    class NextWP(State):
        def __init__(self):
            super().__init__(outcomes=["wp2", "wp3", "wp1"])
        
        def execute(self, bb):
            if not hasattr(bb, "wp_index"):
                bb.wp_index = 0
            bb.wp_index = (bb.wp_index + 1) % 3
            return ["wp2", "wp3", "wp1"][bb.wp_index]

    next_wp = NextWP()

    

    sm = StateMachine(outcomes=["end"])

    # Manejo completo de outcomes
    sm.add_state("WP1", wp1, transitions={
        "succeeded": "CHECK1",
        "aborted": "CHECK1",   
        "canceled": "CHECK1"    
    })
    sm.add_state("CHECK1", check, transitions={"not_found": "WP2", "found": "HOME"})
    
    sm.add_state("WP2", wp2, transitions={
        "succeeded": "CHECK2",
        "aborted": "CHECK2",
        "canceled": "CHECK2"
    })
    sm.add_state("CHECK2", check, transitions={"not_found": "WP3", "found": "HOME"})
    
    sm.add_state("WP3", wp3, transitions={
        "succeeded": "CHECK3",
        "aborted": "CHECK3",
        "canceled": "CHECK3"
    })
    sm.add_state("CHECK3", check, transitions={"not_found": "WP1", "found": "HOME"})

    sm.add_state("HOME", home, transitions={
        "succeeded": "end",
        "aborted": "end",
        "canceled": "end"
    })

    sm.set_start_state("WP1")


    try:
        print("=" * 60)
        print("INICIANDO FSM DE NAVEGACIÓN")
        print("Waypoints: WP1→WP2→WP3 (bucle)")
        print("Detectará verde y volverá a HOME")
        print("=" * 60)
        outcome = sm.execute(bb)
        print(f"\n{'=' * 60}")
        print(f"FSM FINALIZADA - Outcome: {outcome}")
        print(f"{'=' * 60}")
    except KeyboardInterrupt:
        print("\nInterrupción por teclado - Deteniendo...")
    finally:
        executor.shutdown()
        listener.destroy_node()
        rclpy.shutdown()
        print("Sistema cerrado correctamente")

        

if __name__ == "__main__":
    main()