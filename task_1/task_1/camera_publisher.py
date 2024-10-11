import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/processed_camera/image_raw', 10) 
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/gazebo/camera/image_raw',
            self.camera_callback,
            10)

    def camera_callback(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image to /processed_camera/image_raw')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
