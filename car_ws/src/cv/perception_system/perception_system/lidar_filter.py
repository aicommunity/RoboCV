import rclpy
from math import cos, pi
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import MarkerArray
from cv_msg.msg import Object, ObjectList, ClassList
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
# import numpy as np



class LidarFilter(Node):

    def __init__(self):
        super().__init__('lidar_filter')

        qos_policy = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=5)
         

        self.publisher = self.create_publisher(PointCloud2, 'filtered_cloud', qos_policy)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/lidar',
            self.listener_callback,
            qos_profile=qos_policy)


    def listener_callback(self, msg):
        cloud = msg
        gen = point_cloud2.read_points_list(cloud, skip_nans=True, field_names=("x", "y", "z", "intensity"))
        filtered_msg = point_cloud2.create_cloud(msg.header, msg.fields, gen)
        self.publisher.publish(filtered_msg)
        

    
def main(args=None):
    rclpy.init(args=args)

    lidar_filter = LidarFilter()

    rclpy.spin(lidar_filter)

    lidar_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
