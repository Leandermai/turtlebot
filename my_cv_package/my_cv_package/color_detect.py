import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

class ImageInterpreter(Node):
    image_width = 1920
    image_height = 1080
    line_height = 1030  # at this height the ideal center is 1040
    scan_start = 480
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
            Vector3,
            'line_center',
            2)
    
    def listener_callback(self, data):
        # Get image
        # self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)

        # Read
        point_list = []
        for x in range(self.image_width - 1):
            if(x >= self.scan_start):
                if (current_frame[self.line_height, x][2] >= 225):
                    point_list.append(x)

        # Paint
        cv2.line(current_frame, (self.scan_start, self.line_height), (self.image_width-1, self.line_height), (0, 222, 0), 1)
        for x in point_list:
            cv2.circle(current_frame, (x, self.line_height), 10, (0, 125, 255), -1)
        if (len(point_list) != 0):
            center = sum(point_list) / len(point_list)
        else:
            # TODO
            center = float(self.image_width)
        cv2.circle(current_frame, (int(center), self.line_height), 15, (0, 0, 250), -1)

        # Publish
        msg = Vector3()
        msg.x = center
        # self.get_logger().info(f'Difference: {msg.x}')
        self.publisher.publish(msg)

        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_interpreter = ImageInterpreter()
    rclpy.spin(image_interpreter)
    image_interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
