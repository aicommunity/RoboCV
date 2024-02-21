import rclpy
import colorsys
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from message_filters import TimeSynchronizer, Subscriber
from rclpy.node import Node
from math import pi, cos, sin

from sensor_msgs.msg import CameraInfo, Image
from cv_msg.msg import Object, ObjectList, ClassList


class Vizualization(Node):

    def __init__(self):
        super().__init__('vizualization')

        self.sub_img = Subscriber(self, Image, "/carla/ego_vehicle/semantic_segmentation_front/image")
        self.sub_res = Subscriber(self, ClassList, "/tracker_out")

        self.tss = TimeSynchronizer([self.sub_img, self.sub_res], 10)
        self.tss.registerCallback(self.img_callback)
        
        self.cv_bridge = CvBridge()
        self.res_msg = ClassList()

        self.hsv_img = ''

        self.focal_length = 200  # in px
        self.cam_h = 70
        self.cam_w = 400
        self.cam_fov = pi/2

        self.te = 0
        self.st = 0
    
    def draw_map(self, angle, dist, color):
        dist_scale = 5
        angle_scale = 1

        x = int(round(sin(angle_scale*angle+pi), 2)*dist*dist_scale + 256)
        y = int(round(cos(angle_scale*angle+pi), 2)*dist*dist_scale + 256)
        cv.circle(self.map, (x,y), 2, color, -1)
        print(x,y)

    def res_parsing(self):
        counter = 0
        classes = self.res_msg
        for cls in classes.class_objs: # Для каждого класса объектов в листе классов
            for obj in cls.objects:  # Для каждого объекта в листе объектов одного класса
                (b ,g, r) = (11*cls.id, 11*cls.id, 11*cls.id)
                x = obj.img_x
                y = obj.img_y
                w = obj.img_w
                h = obj.img_h

                dist = obj.dist
                angle = obj.angle

                cv.rectangle(self.hsv_img, (x, y), (x + w, y + h), (b, g, r), 1)
                cv.putText(self.hsv_img, str(cls.id) + " " + str(obj.id), (x, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 1)

                (x1_l, y1_l) = (int(self.cam_w/2), self.cam_h)
                (x2_l, y2_l) = (int(x+w//2), int(y+h//2))
                cv.line(self.hsv_img, (x1_l,y1_l), (x2_l,y2_l), (b, g, r), 1)

                self.draw_map(angle, dist, (b, g, r))
                
                counter += 1


        

    def img_callback(self, img, res):
        self.map = np.zeros((512,512,3), np.uint8)
        cv.circle(self.map,(256,256), 256, (0,0,255), 1)
        cv.rectangle(self.map,(246,236),(266,276),(0,255,0),2)
        cv.line(self.map, (256, 256), (0,0), (0,255,0), 2)
        cv.line(self.map, (256, 256), (512, 0), (0,255,0), 2)

        self.res_msg = res
        cv_img = self.cv_bridge.imgmsg_to_cv2(img, 'bgra8')
        self.hsv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)

        self.res_parsing()
        height, width, _ = self.hsv_img.shape
        resize_points = (width*2, height*2)
        resized = cv.resize(self.hsv_img, resize_points, interpolation= cv.INTER_LINEAR)

        cv.imshow('radar', self.map)
        cv.imshow("res", resized)
        cv.waitKey(1)
        

        

    

def main(args=None):
    rclpy.init(args=args)

    vizualization = Vizualization()

    rclpy.spin(vizualization)

    vizualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
