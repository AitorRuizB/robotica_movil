import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros

class LaserMapper(Node):
    def __init__(self):
        super().__init__('laser_mapper')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)  # usa el clock del nodo
        self.create_timer(1, self.tick)

        #suscripcion al scan
        self.scan_sub =self.create_subscription(LaserScan,'/scan_raw',self.scan_callback,10)

        #publisher pointcloud

        self.cloud_pub=self.create_publisher(PointCloud2,'laser_cloud',10)
    def tick(self):
        try:
            tf = self.buffer.lookup_transform('odom', 'scan_raw', rclpy.time.Time())
            p_in = PointStamped()
            p_in.header.frame_id = 'scan_raw'
            #Pasamos un punto cualquiera, es solo un ejemplo
            p_in.point.x, p_in.point.y, p_in.point.z = 1.0, 0.0, 0.0
            p_out = do_transform_point(p_in, tf)
            print(f'{p_in.point} en scan_raw es {p_out.point} en odom')
        except (tf2_ros.TransformException,) as ex:
            self.get_logger().warn(f'TF2 error: {ex}')

def main():
    rclpy.init()
    node = LaserMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()