import rclpy
from math import pi
from rclpy.node import Node
from ..config import config

from cv_msg.msg import ClassList

size_dict = {  # Метры, w,h. Словарь с рамзерами объектов. Заполнядся вручную из списка маркеров объектов
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
                        10: (0, 2.000),  # Vehicles, Works
                        11: (0, 0),
                        12: (0, 0.682),  # TrafficSigns, 
                        13: (0, 0),
                        14: (0, 0),
                        15: (0, 0),
                        16: (0, 0),
                        17: (0, 0),
                        18: (0.277, 1.221),  # TrafficLights, Works
                        19: (0, 0),
                        20: (0, 0),
                        21: (0, 0),
                        22: (0, 0)
                        }


class FakeLocalization(Node):  # Класс локализации объектов. Работает по колбэку.

    def __init__(self):
        super().__init__('fake_localization')
        self.publisher = self.create_publisher(ClassList, 'localization_out', 10)
        self.subscription = self.create_subscription(
            ClassList,
            '/detector_out',
            self.listener_callback,
            10)
        
        self.focal_length = config.cam_focal_length
        self.cam_h = config.cam_height
        self.cam_w = config.cam_width
        self.cam_fov = config.cam_fov

    def calculate_dist(self):  # Вычисление расстояния
        dist = (self.focal_length * self.object_height) / self.object_img_height
        return dist

    def calculate_angle(self):  # Вычисление угла
        rad_per_pixel = self.cam_fov / self.cam_w
        obj_center = self.object_img_x_pos + self.object_img_width/2
        angle = -(obj_center - self.cam_w/2) * rad_per_pixel
        return angle
        
    def listener_callback(self, msg):
        classes = msg
        for cls in classes.class_objs: # Для каждого класса объектов в листе классов
                self.object_height = size_dict[cls.id][1]
                for obj in cls.objects:  # Для каждого объекта в листе объектов одного класса
                    self.object_img_height = obj.img_h
                    self.object_img_x_pos = obj.img_x
                    self.object_img_width = obj.img_w
                    
                    obj.dist = self.calculate_dist()
                    obj.angle = self.calculate_angle()

                    if config.global_debug or config.localization_debug:
                        print(classes)
        self.publisher.publish(classes)

    
def main(args=None):
    rclpy.init(args=args)

    fake_localizator = FakeLocalization()

    rclpy.spin(fake_localizator)

    fake_localizator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

