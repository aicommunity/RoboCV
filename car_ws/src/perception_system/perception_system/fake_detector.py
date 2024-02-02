import rclpy
from rclpy.node import Node
import carla

from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

class FakeDetector(Node):

    def __init__(self):
        super().__init__('fake_detector')
        self.publisher_ = self.create_publisher(Image, 'semantic_image', 10)
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        # msg.data.convert(carla.ColorConverter.CityScapesPalette)
        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)

    fake_detector = FakeDetector()

    rclpy.spin(fake_detector)

    rclpy.spin(fake_detector)

    fake_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()