import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PassTunnel(Node):
    def __init__(self):
        super().__init__('pass_tunnel')
        self.get_logger().info('Hello from pass_tunnel')

    # Variables
        # Flags
        self.block = True

        # Sensor data
        self.min_angle = None
        self.latest_frame = None

        # Config
        self.rotation_factor = 0.2


    # subscriber
        # State
        self.subscriber = self.create_subscription(
            Int32, 'state', self.state_callback, 3)

        # Image
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 3)
        self.br = CvBridge()

        # LIDAR
        self.subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)


    # Publisher
        # response
        self.pub_response = self.create_publisher(Int32, 'response', 3)
        
        # Turtlebot
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # Main logic callback
        self.timer = self.create_timer(0.1, self.timer_callback)


# Callbacks
    def image_callback(self, msg):
        self.latest_frame = self.br.imgmsg_to_cv2(msg)

    def laser_callback(self, msg):
        points = msg.ranges
        min_val = 99.9     # absurd number as starting point
        for x in range(len(points)):
            if (points[x] < min_val):
                min_val = points[x]
                self.min_angle = x

    def state_callback(self, command):
        msg = Int32()
        if command.data == 4:
            self.block = False
            msg.data = 41
        else:
            self.block = True
            msg.data = 40
        self.pub_response.publish(msg)


    # Main Logic
    def timer_callback(self):
        if self.block:
            return

        msg = Twist()
        msg.angular.z = self.calculateRotation(self.min_angle, 90)
        msg.angular.z *= self.rotation_factor
        msg.linear.x = 0.175

        self.get_logger().warn(f'Forward: {msg.linear.x}, Rotation: {msg.angular.z:.3f}')
        self.pub_vel.publish(msg)


    # Calculate the amount of rotation needed to stand 90° to the minAngle point
    def calculateRotation(self, angle, target, factor=8):
        target = (-target - 180) % 360
        return (((angle - target) % 360) / (180/factor)) - factor
