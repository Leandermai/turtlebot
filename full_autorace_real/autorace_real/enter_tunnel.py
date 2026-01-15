import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class EnterTunnel(Node):
    def __init__(self):
        super().__init__('enter_tunnel')
        self.get_logger().info('Hello from enter_tunnel')

    # Variables
        # Flags
        self.block = True
        self.state = 0

        # Sensor data
        self.latest_frame = None
        self.is_laser_right = False
        self.is_laser_front_right = False

        # Config
        self.scan_start = 480
        self.line_height = 1030
        self.ideal_center = 1440
        self.max_correction = 960


    # subscriber
        # State
        self.subscriber = self.create_subscription(
            Int32, 'state', self.response_callback, 3)

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
        self.is_laser_right = min(msg.ranges[265:275]) <= 2.0
        self.is_laser_front_right = min(msg.ranges[320:330]) <= 3.5

    def response_callback(self, msg):
        response = Int32()
        if msg.data == 3:
            self.block = False
            response.data = 31
        else:
            self.block = True
            response.data = 30
        self.pub_response.publish(response)


    # Main Logic
    def timer_callback(self):
        if self.block:
            return

        if self.state == 0:
            self.follow_line()

            if self.is_laser_front_right:
                self.state = 1

        else:
            if self.is_laser_right:
                msg = Int32()
                msg.data = 32
                self.pub_response.publish(msg)
            else:
                msg = Twist()
                msg.angular.z = 0.0
                msg.linear.x = 0.25
                self.get_logger().warn(f'Velocity: {msg.linear.x}')
                self.pub_vel.publish(msg)

# Helper functions
    def follow_line(self):
        point_list = []
        height, width = self.latest_frame.shape[:2]

        for x in range(width - 1):
            if(x >= self.scan_start):
                if (self.latest_frame[self.line_height, x][2] >= 225):
                    point_list.append(x)

        if (len(point_list) != 0):
            self.line_center = sum(point_list) / len(point_list)

        else:
            self.line_center = float(width)

        msg = Twist()
        correction = self.ideal_center - self.line_center
        if (correction < 0):
            correction *= 2 # normalize the left and right error to be equal in total magnitude
        correction /= (self.max_correction / 2.0)   # makes the angular.z max out at 2 for both sides
        
        if (correction > 0.9):
            correction = 0.9
            
        msg.angular.z = correction
        msg.linear.x = 0.1
        self.get_logger().warn(f'Velocity: {msg.linear.x:.3f}, Correction: {msg.angular.z:.3f}')
        self.pub_vel.publish(msg)
    