import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageInterpreter(Node):
    isInLine = False
    def __init__(self):
        super().__init__('image_interpreter')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            2)
        self.br = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("camera", 640, 480)

        self.publisher = self.create_publisher(
            Bool,
            'back_in_line',
            2)
        self.new_image = False
        self.timer = self.create_timer(0.005, self.timer_callback)
    
    def listener_callback(self, data):
        # Get image
        self.latest_frame = self.br.imgmsg_to_cv2(data)
        self.new_image = True

    def timer_callback(self):
        # Check new Data
        if not self.new_image:
            return
        self.new_image = False
        current_frame = self.latest_frame

        # Read
        if (current_frame[960, 810][2] >= 225):
            self.isInLine = True
        else:
            self.isInLine = False

        # Paint
        if (self.isInLine):
            cv2.circle(current_frame, (960, 810), 20, (60, 90, 120), -1)
        else:
            cv2.circle(current_frame, (960, 810), 10, (0, 120, 255), -1)

        # Publish
        msg = Bool()
        msg.data = self.isInLine
        self.publisher.publish(msg)
        if self.isInLine:
            self.get_logger().info(f'Line detected')

        # Display
        #current_frame = cv2.resize(current_frame, (0,0), fx=0.1, fy=0.1)
        #cv2.imshow("camera", current_frame)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_interpreter = ImageInterpreter()
    rclpy.spin(image_interpreter)
    image_interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
