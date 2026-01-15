import cv2
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AvoidObstacle(Node):
    def __init__(self):
        super().__init__('avoid_obstacle')
        self.get_logger().info('Hello from avoid_obstacle')


    # Variables
        # Flags
        self.state = 0
        self.isOnLine = False

        # Sensor data
        self.latest_frame = None
        self.min_angle = None

        # Config
        self.adjust_time = 1
        self.time = 0
        self.block = True
    

    # Subscribers
        # State
        self.subscriber = self.create_subscription(
            Int32, 'state', self.response_callback, 3)

        # Get image
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 2)
        self.br = CvBridge()

        # Get Laser
        self.subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)


    # Publish
        # States
        self.pub_command = self.create_publisher(Int32, 'response', 3)
        
        # Turtlebot
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # Main logic callback
        self.timer = self.create_timer(0.1, self.timer_callback)


# Callbacks


    # Get image
    def image_callback(self, data):
        self.latest_frame = self.br.imgmsg_to_cv2(data)

    # read the direction of the closest laser
    def laser_callback(self, msg):  # Range: 0.12 - 3.5
        self.ranges = msg.ranges
        min_val = 99.9     # absurd number as starting point
        for x in range(len(self.ranges)):
            if (self.ranges[x] < min_val):
                min_val = self.ranges[x]
                self.min_angle = x


    def response_callback(self, msg):
        response = Int32()
        if msg.data == 2:
            self.block = False
            response.data = 21
        else:
            self.block = True
            response.data = 20
        self.pub_command.publish(response)


    # Main logic
    def timer_callback(self):
        if self.block:
            return

        msg = Twist()

        self.checkImage()

        if (self.state == 0):
            if self.isOnLine:
                self.time = 0
                self.get_logger().warn('Aligning back on track')
                self.state = 1
                return

            msg.angular.z = self.calculateRotation(self.min_angle, 60)
            msg.linear.x = 0.175

        else:
            if self.time >= self.adjust_time:
                msg = Int32()
                msg.data = 22
                self.pub_command.publish(msg)
                return
            msg.angular.z = self.calculateRotation(self.min_angle, 180)
            msg.linear.x = 0.0

            self.time += 0.1

        self.get_logger().warn(f'Forward: {msg.linear.x}, Rotation: {msg.angular.z:.3f}')
        self.pub_vel.publish(msg)


    def checkImage(self):
        if (self.latest_frame[960, 810][2] >= 225):
            isInLine = True
        else:
            isInLine = False

        if isInLine:
            self.isOnLine = True
        

    # Calculate the amount of rotation needed to stand 90° to the minAngle point
    def calculateRotation(self, angle, target, factor=8):
        target = (-target - 180) % 360
        return (((angle - target) % 360) / (180/factor)) - factor

    