import rclpy
import colorsys
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from cv_msg.msg import Object, ObjectList, ClassList


class Vizualization(Node):

    def __init__(self):
        super().__init__('vizualization')
        self.sub_img = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.img_callback,
            10)
        
        self.sub_res = self.create_subscription(
            ClassList,
            '/detector_out',
            self.res_callback,
            10)
        
        self.cv_bridge = CvBridge()
        self.hsv_img = ''
        self.is_res = 0 
        self.res_msg = ClassList()
            
    # def listener_callback2(self, msg):
    #     self.class_list = ClassList()

    #     cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgra8')
    #     self.hsv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
    #     height, width, _ = self.hsv_img.shape

    #     # for h in range(height):
    #     #     for w in range(width):
    #     #         print(self.hsv_img[h][w])

    #     self.hsv_img, single_class_list = self.lights.process(self.hsv_img)
    #     self.class_list.class_objs.append(single_class_list)

    #     self.hsv_img, single_class_list = self.signs.process(self.hsv_img)
    #     # self.c
    #     # lass_list.class_objs.append(single_class_list)
    def res_parsing(self):
        if self.is_res:
            counter = 0
            classes = self.res_msg
            for cls in classes.class_objs: # Для каждого класса объектов в листе классов
                for obj in cls.objects:  # Для каждого объекта в листе объектов одного класса
                    x = obj.img_x
                    y = obj.img_y
                    w = obj.img_w
                    h = obj.img_h

                    cv.rectangle(self.hsv_img, (x, y), (x + w, y + h), (11*cls.id, 11*cls.id, 11*cls.id), 1)
                    cv.putText(self.hsv_img, str(cls.id), (x, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 1)
                    counter += 1
            self.is_res = 0 


        

    def img_callback(self, msg):
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgra8')
        self.hsv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)

        self.res_parsing()
        height, width, _ = self.hsv_img.shape
        resize_points = (width*2, height*2)
        resized = cv.resize(self.hsv_img, resize_points, interpolation= cv.INTER_LINEAR)

        cv.imshow("res", resized)
        cv.waitKey(1)
        

    def res_callback(self, msg):
        self.is_res = 1
        self.res_msg = msg  

        

    

def main(args=None):
    rclpy.init(args=args)

    vizualization = Vizualization()

    rclpy.spin(vizualization)

    vizualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
