import rclpy
import colorsys
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from cv_msg.msg import Object, ObjectList, ClassList

hsv_dict = {
                        0: (0, 0, 0),
                        1: (0, 0, 70),  # Buildings, Works
                        2: (0, 153, 100),
                        3: (80, 99, 90),
                        4: (173, 231, 220),
                        5: (0, 0, 153),
                        6: (42, 200, 234),
                        7: (149, 127, 128),
                        8: (150, 218, 244),
                        9: (39, 192, 142),
                        10: (119, 255, 142),
                        11: (119, 88, 156),
                        12: (30, 255, 220),  # TrafficSigns, 
                        13: (103, 155, 180),
                        14: (149, 255, 81),
                        15: (0, 85, 150),
                        16: (3, 99, 230),
                        17: (149, 21, 180),
                        18: (19, 224, 250),  # TrafficLights, Works
                        19: (78, 107, 190),
                        20: (17, 180, 170),
                        21: (115, 178, 150),
                        22: (40, 105, 170)
                        }

def rgb2hsv(ri, gi, bi):
    # Normalize
    (r, g, b) = (ri / 255, gi / 255, bi / 255)

    # Convert to hsv
    (h, s, v) = colorsys.rgb_to_hsv(r, g, b)

    # Expand HSV range
    return (int(h * 179), int(s * 255), int(v * 255))



class ClassDetector():
    def __init__(self, obj_name, obj_class):
        self.obj_name = obj_name
        self.obj_class = obj_class
        self.src = ""
    
    def get_segmentation_mask(self, show_res=1):
        self.seg_mask = cv.inRange(self.src, hsv_dict[self.obj_class], hsv_dict[self.obj_class])
        if show_res:
            cv.imshow("seg mask of " + str(self.obj_name), self.seg_mask)  # DEBUG
            cv.waitKey(1)  # DEBUG
    
    def get_rect(self):
        self.obj_list = ObjectList(tag=self.obj_name, id=self.obj_class)
        contours, hierarchy = cv.findContours(self.seg_mask, 1, 2)
        for i in contours:
            moments = cv.moments(i, 1)
            if moments['m00'] > 5:  # Min square check
                x,y,w,h = cv.boundingRect(i)
                # cv.rectangle(self.src, (x, y), (x + w, y + h), (11*self.obj_class, 11*self.obj_class, 11*self.obj_class), 1)
                # cv.putText(self.src, str(self.obj_class), (x, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 1)

                obj = Object()
                obj.img_x = x
                obj.img_y = y
                obj.img_w = w
                obj.img_h = h
                obj.rel_det = 1.0
                self.obj_list.objects.append(obj)

    def process(self, img):
        self.src = img
        self.get_segmentation_mask()
        self.get_rect()
        return self.src, self.obj_list

    

class FakeDetector(Node):

    def __init__(self):
        super().__init__('fake_detector')
        self.publisher_ = self.create_publisher(ClassList, 'detector_out', 10)
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.listener_callback,
            10)
        self.cv_bridge = CvBridge()

        self.hsv_img = ''
        
        self.lights = ClassDetector("TrafficLight", 18)
        self.signs = ClassDetector("TrafficSigns", 12)
        

        
    def listener_callback(self, msg):
        self.class_list = ClassList()

        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgra8')
        self.hsv_img = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
        height, width, _ = self.hsv_img.shape

        # for h in range(height):
        #     for w in range(width):
        #         print(self.hsv_img[h][w])

        self.hsv_img, single_class_list = self.lights.process(self.hsv_img)
        self.class_list.class_objs.append(single_class_list)

        self.hsv_img, single_class_list = self.signs.process(self.hsv_img)



        # up_points = (width*3, height*3)
        # resized = cv.resize(self.hsv_img, up_points, interpolation= cv.INTER_LINEAR)
        # cv.imshow("res", resized)
        # cv.waitKey(1)
        self.publisher_.publish(self.class_list)

    


    

def main(args=None):
    rclpy.init(args=args)

    fake_detector = FakeDetector()

    rclpy.spin(fake_detector)

    fake_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






# self.hsv_dict = {
#                         0: (0, 0, 0): 0,
#                         1: (0, 0, 70): 1,
#                         2: (0, 153, 100): 2,
#                         3: (80, 99, 90): 3,
#                         4: (173, 231, 220): 4,
#                         5: (0, 0, 153): 5,
#                         6: (42, 200, 234): 6,
#                         7: (149, 127, 128): 7,
#                         8: (150, 218, 244): 8,
#                         9: (39, 192, 142): 9,
#                         10: (119, 255, 142): 10,
#                         11: (119, 88, 156): 11,
#                         12: (29, 255, 220): 12,
#                         13: (103, 155, 180): 13,
#                         14: (149, 255, 81): 14,
#                         15: (0, 85, 150): 15,
#                         16: (3, 99, 230): 16,
#                         17: (149, 21, 180): 17,
#                         18: (18, 224, 250): 18,
#                         19: (78, 107, 190): 19,
#                         20: (17, 180, 170): 20,
#                         21: (115, 178, 150): 21,
#                         22: (40, 105, 170): 22,
#                         }