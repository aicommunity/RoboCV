import rclpy
from math import cos, pi
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import MarkerArray
from cv_msg.msg import Object, ObjectList, ClassList
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import cv2 as cv
import numpy as np



class FakeLidar(Node):

    def __init__(self):
        super().__init__('fake_lidar')

        qos_policy = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=5)
         

        # self.publisher = self.create_publisher(ClassList, 'tracker_out', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/ego_vehicle/lidar',
            self.listener_callback,
            qos_profile=qos_policy)


 
         
    def listener_callback(self, msg):
        self.map = np.zeros((512,512,3), np.uint8)
        cv.circle(self.map,(256,256), 256, (0,0,255), 1)
        # cv.rectangle(self.map,(246,236),(266,276),(0,255,0),2)
        # cv.line(self.map, (256, 256), (0,0), (0,255,0), 2)
        # cv.line(self.map, (256, 256), (512, 0), (0,255,0), 2)

        # classes = msg
        # for cls in classes.class_objs: # Для каждого класса объектов в листе классов
        #     for obj in cls.objects:  # Для каждого объекта в листе объектов одного класса
        #         pass

        # self.publisher.publish(classes)

        dist_scale = 5
        # angle_scale = 1

        cloud = msg
        gen = point_cloud2.read_points_list(cloud, skip_nans=True, field_names=("x", "y", "z"))
        for point in gen:
            cv.circle(self.map, (int(point.x*dist_scale) + 256, int(point.y*dist_scale) + 256), 2, (0,0,255), -1)

        # with open("/home/eddyswens/ROS/RoboCV/car_ws/src/cv/perception_system/lidar_result.txt", "w") as file:
        #     # for line in gen:
        #     file.write(str(msg) + '\n')
        cv.imshow('radar', self.map)
        cv.waitKey(1)

    
def main(args=None):
    rclpy.init(args=args)

    fake_lidar = FakeLidar()

    rclpy.spin(fake_lidar)

    fake_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

