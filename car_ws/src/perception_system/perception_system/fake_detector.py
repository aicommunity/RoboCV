import rclpy
import colorsys
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image

def rgb2hsv(ri, gi, bi):
    # Normalize
    (r, g, b) = (ri / 255, gi / 255, bi / 255)

    # Convert to hsv
    (h, s, v) = colorsys.rgb_to_hsv(r, g, b)

    # Expand HSV range
    return (int(h * 179), int(s * 255), int(v * 255))

class FakeDetector(Node):

    def __init__(self):
        super().__init__('fake_detector')
        self.publisher_ = self.create_publisher(Image, 'semantic_image', 10)
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.listener_callback,
            10)
        self.cv_bridge = CvBridge()

        self.cv_img = ''
        self.hsv_dict = {
                        (0, 0, 0): 0,
                        (0, 0, 70): 1,
                        (0, 153, 100): 2,
                        (80, 99, 90): 3,
                        (173, 231, 220): 4,
                        (0, 0, 153): 5,
                        (42, 200, 234): 6,
                        (149, 127, 128): 7,
                        (152, 218, 244): 8,
                        (39, 192, 142): 9,
                        (119, 255, 142): 10,
                        (120, 88, 156): 11,
                        (29, 255, 220): 12,
                        (103, 155, 180): 13,
                        (149, 255, 81): 14,
                        (0, 85, 150): 15,
                        (3, 99, 230): 16,
                        (149, 21, 180): 17,
                        (18, 224, 250): 18,
                        (78, 107, 190): 19,
                        (17, 180, 170): 20,
                        (115, 178, 150): 21,
                        (41, 105, 170): 22,
                        }
        
    def listener_callback(self, msg):
        self.cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgra8')
        hsv_img = cv.cvtColor(self.cv_img, cv.COLOR_BGR2HSV)
        height, width, _ = hsv_img.shape

        # print(hsv_img.shape)
        # res = cv.inRange(hsv_img, rgb2hsv(250, 170, 30), (19, 224, 250))

        # print(hsv_img[60][30])

        for h in range(height):
            for w in range(width):
                print(hsv_img[h][w])

        cv.imshow("raw msg", hsv_img)
        cv.waitKey(1)
        # self.publisher_.publish(self.msg)

    


    

def main(args=None):
    rclpy.init(args=args)

    fake_detector = FakeDetector()

    rclpy.spin(fake_detector)

    fake_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()