import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from rclpy.time import Time


class LandmarkMapper(Node):
    PEAK_THRESHOLD = 1.50 #Distancia pico
    CONE_MAX_SIZE = 0.2 
    CONTAINER_MIN_SIZE = 0.2
    CONTAINER_MAX_SIZE = 2.0
    DUPLICATE_THRESHOLD = 2.5 #Radio de landmark

    def __init__(self):
        super().__init__('landmark_mapper')
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/landmarks', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.landmarks = []

    def scan_callback(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except tf2_ros.TransformException:
            return

        detected = self.detect_landmarks_from_scan(msg, tf)
        for gx, gy, mtype in detected:
            if not self.is_near_existing(gx, gy):
                self.landmarks.append((gx, gy, mtype))
                self.get_logger().info(f"Nuevo landmark en ({gx:.2f}, {gy:.2f}) tipo {mtype}")
        self.publish_landmarks()

    def detect_landmarks_from_scan(self, scan, tf):
        #Comprobaciones para saber si es landmark
        ranges = scan.ranges
        angle = scan.angle_min
        points = []
        for r in ranges:
            if math.isinf(r) or math.isnan(r) or r <= 0.1:
                points.append(None)
            else:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += scan.angle_increment

        landmarks = []
        i = 1
        in_object = False
        start_point = None
        waiting_for_rise = False
        #Comprobaciones de pico y ancho
        while i < len(points):
            p_prev = points[i - 1]
            p_curr = points[i]
            i += 1
            if p_prev is None or p_curr is None:
                continue
            if not in_object and (math.hypot(p_prev[0], p_prev[1]) - math.hypot(p_curr[0], p_curr[1])) > self.PEAK_THRESHOLD:
                in_object = True
                waiting_for_rise = True
                start_point = p_curr
            elif in_object and waiting_for_rise and (math.hypot(p_curr[0], p_curr[1]) - math.hypot(p_prev[0], p_prev[1])) > self.PEAK_THRESHOLD:
                end_point = p_prev
                length = math.hypot(end_point[0] - start_point[0], end_point[1] - start_point[1])
                if length <= self.CONE_MAX_SIZE:
                    marker_type = Marker.CYLINDER
                elif self.CONTAINER_MIN_SIZE <= length <= self.CONTAINER_MAX_SIZE:
                    marker_type = Marker.CUBE
                else:
                    in_object = False
                    waiting_for_rise = False
                    start_point = None
                    continue
                lm = self.process_landmark(start_point, end_point, scan, tf, marker_type)
                if lm:
                    landmarks.append(lm)
                in_object = False
                waiting_for_rise = False
                start_point = None
        return landmarks

    def process_landmark(self, start_point, end_point, scan, tf, marker_type):
        #Conversion de landmark a coordenadas globales
        x_local = (start_point[0] + end_point[0]) / 2.0
        y_local = (start_point[1] + end_point[1]) / 2.0
        p_in = PointStamped()
        p_in.header.frame_id = scan.header.frame_id
        p_in.header.stamp = scan.header.stamp
        p_in.point.x = x_local
        p_in.point.y = y_local
        p_in.point.z = 0.0
        try:
            p_global = do_transform_point(p_in, tf)
            return (p_global.point.x, p_global.point.y, marker_type)
        except Exception:
            return None

    def is_near_existing(self, x, y):
        #Comprobacion de radio por error de odom
        for lx, ly, _ in self.landmarks:
            if math.hypot(lx - x, ly - y) < self.DUPLICATE_THRESHOLD:
                return True
        return False

    def publish_landmarks(self):
        #Publica el landmark dependiendo del tipo si es cono o contenedor
        marker_array = MarkerArray()
        for i, (x, y, mtype) in enumerate(self.landmarks):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'landmarks'
            m.id = i
            m.type = mtype
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2 if mtype == Marker.CYLINDER else 0.8
            m.scale.y = 0.2 if mtype == Marker.CYLINDER else 1.0
            m.scale.z = 0.5
            m.color.a = 1.0
            m.color.r = 0.0 if mtype == Marker.CUBE else 0.0
            m.color.g = 1.0 if mtype == Marker.CYLINDER else 0.5
            m.color.b = 0.3 if mtype == Marker.CYLINDER else 1.0
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = LandmarkMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
