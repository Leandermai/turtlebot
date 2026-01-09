import cv2
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Obstacle_avoider(Node):
    state = 0
    isOnLine = False
    new_image = False
    def __init__(self):
        super().__init__('avoider')
        ranges = None

        # Get image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            2)
        self.br = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("camera", 640, 480)

        # Get Laser
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Tell the bot to move
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    # Get image
    def image_callback(self, data):
        self.latest_frame = self.br.imgmsg_to_cv2(data)
        self.new_image = True

    minAngle = -1
    # read the direction of the closest laser
    def laser_callback(self, msg):  # Range: 0.12 - 3.5
        self.ranges = msg.ranges

        self.ranges = msg.ranges
        min_val = 999.9
        for x in range(len(self.ranges)):
            if (self.ranges[x] < min_val):
                min_val = self.ranges[x]
                self.minAngle = x
        self.get_logger().info(f'Closest degree: {self.minAngle} in distance {min_val:.2f}')

    def timer_callback(self):
        msg = Twist()

        self.checkImage()

        if self.minAngle == -1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f'404 Wall not found')
            return

        if self.isOnLine:
            self.state = 1

        if (self.state == 0):
            msg.angular.z = self.calculateRotation(self.minAngle, 60)
            msg.linear.x = 0.2

            self.get_logger().info(f'Forward: {msg.linear.x}, Rotation: {msg.angular.z:.3f}')
        else:
            msg.angular.z = self.calculateRotation(self.minAngle, 180)
            msg.linear.x = 0.0

        self.publisher.publish(msg)


    def checkImage(self):
        # Check new Data
        if not self.new_image:
            return
        self.new_image = False
        current_frame = self.latest_frame

        # Read
        if (current_frame[960, 810][2] >= 225):
            isInLine = True
        else:
            isInLine = False

        # Paint
        if (isInLine):
            cv2.circle(current_frame, (960, 810), 20, (60, 90, 120), -1)
        else:
            cv2.circle(current_frame, (960, 810), 10, (0, 120, 255), -1)
        
        if isInLine:
            self.isOnLine = True
        

    # Calculate the amount of rotation needed to stand 90° to the minAngle point
    def calculateRotation(self, angle, target, factor=8):
        target = (-target - 180) % 360
        return (((angle - target) % 360) / (180/factor)) - factor

def main(args=None):
    rclpy.init(args=args)
    avoider = Obstacle_avoider()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()