import rclpy
from math import cos, pi
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import MarkerArray
from cv_msg.msg import Object, ObjectList, ClassList

size_dict = {  # Meters i guess, w, h
                        0: (0, 0),
                        1: (0, 0),  # Buildings, Works
                        2: (0, 0),
                        3: (0, 0),
                        4: (0, 0),
                        5: (0, 0),
                        6: (0, 0),
                        7: (0, 0),
                        8: (0, 0),
                        9: (0, 0),
                        10: (0, 0),
                        11: (0, 0),
                        12: (0, 0),  # TrafficSigns, 
                        13: (0, 0),
                        14: (0, 0),
                        15: (0, 0),
                        16: (0, 0),
                        17: (0, 0),
                        18: (0.277, 0),  # TrafficLights, Works
                        19: (0, 0),
                        20: (0, 0),
                        21: (0, 0),
                        22: (0, 0)
                        }

class FakeLocalization(Node):

    def __init__(self):
        super().__init__('fake_localization')

        # qos_policy = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        #     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #     durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        #     depth=1)
        #  qos_profile=qos_policy

        self.subscription = self.create_subscription(
            ClassList,
            '/detector_out',
            self.listener_callback,
            10)
        
        self.focal_length = 200  # in mm i guess
        self.cam_h = 70
        self.cam_w = 400
        self.cam_fov = pi/2


    def calculate_dist(self):
        dist_normal = (self.focal_length * self.object_width) / self.object_img_width
        return dist_normal

    def calculate_angle(self):
        rad_per_pixel = self.cam_fov / self.cam_w
        obj_center = self.object_img_x_pos + self.object_img_width/2

        angle = -(obj_center - self.cam_w/2) * rad_per_pixel
        return angle
        
    def listener_callback(self, msg):
        classes = msg
        for cls in classes.class_objs: # Для каждого класса объектов в листе классов
                self.object_width = size_dict[cls.id][0]
                for obj in cls.objects:  # Для каждого объекта в листе объектов одного класса
                    self.object_img_x_pos = obj.img_x
                    self.object_img_width = obj.img_w
                    
                    dist_normal = self.calculate_dist()
                    angle = self.calculate_angle()
                    dist = dist_normal / cos(angle)
                    print("Dist: " + str(dist) + "Angle: " + str(angle))

    
def main(args=None):
    rclpy.init(args=args)

    fake_localizator = FakeLocalization()

    rclpy.spin(fake_localizator)

    fake_localizator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

