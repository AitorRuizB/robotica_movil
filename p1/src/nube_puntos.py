import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import math
import struct

class LaserMapper(Node):
    def __init__(self):
        #Suscripciones y publicaciones laser/nube
        super().__init__('laser_mapper')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/laser_cloud', 10)
        self.voxel_map = {}
        self.last_scan_points = []
        self.VOXEL_SIZE = 0.1
        self.create_timer(1.0, self.add_last_scan_to_cloud)

    def scan_callback(self, msg: LaserScan):
        #Lector de datos del laser y transformacion a nube de puntos
        try:
            if not self.buffer.can_transform('odom', msg.header.frame_id, msg.header.stamp):
                return
            tf = self.buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp)
            new_points = []
            angle = msg.angle_min
            for r in msg.ranges:
                if r < msg.range_min or r > msg.range_max:
                    angle += msg.angle_increment
                    continue
                # Pasar de distancias a puntos en SC
                x_local = r * math.cos(angle)
                y_local = r * math.sin(angle)
                p_in = PointStamped()
                p_in.header = msg.header
                p_in.point.x = x_local
                p_in.point.y = y_local
                p_in.point.z = 0.0
                p_global = do_transform_point(p_in, tf)
                new_points.append((p_global.point.x, p_global.point.y, p_global.point.z))
                angle += msg.angle_increment
            self.last_scan_points = new_points
        except (tf2_ros.TransformException,) as ex:
            self.get_logger().warn(f'TF2 error: {ex}')

    def add_last_scan_to_cloud(self):
        #Añadir los puntos a la nube 
        if not self.last_scan_points:
            return
        for x, y, z in self.last_scan_points:
            key = (round(x / self.VOXEL_SIZE), round(y / self.VOXEL_SIZE), round(z / self.VOXEL_SIZE))
            if key not in self.voxel_map:
                self.voxel_map[key] = (x, y, z)
        cloud_points = list(self.voxel_map.values())
        cloud_msg = self.create_pointcloud2(cloud_points, frame_id='odom')
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info(f'Nube voxelizada publicada con {len(cloud_points)} voxels')

    @staticmethod
    def create_pointcloud2(points, frame_id='odom'):
        #Crear nube de puntos y voxelizar
        msg = PointCloud2()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        buffer = bytearray()
        for x, y, z in points:
            buffer.extend(struct.pack('fff', x, y, z))
        msg.data = buffer
        return msg

def main():
    rclpy.init()
    node = LaserMapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
