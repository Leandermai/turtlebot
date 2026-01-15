import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class FollowLine(Node):
    def __init__(self):
        super().__init__('follow_line')
        self.get_logger().info('Hello from follow_line')


    # Variables
        # Flag
        self.block = True

        # Sensor data
        self.latest_frame = None
        self.line_center = None

        # Config
        self.scan_start = 480
        self.line_scan_y = 1030
        self.max_correction = 960
        self.ideal_center_x = 1440
        self.velocity_step = 0.025
        self.min_velocity = 0.1
        self.velocity = 0.1
        self.max_velocity = 0.4
        self.lost_turn_rate = -0.5


    # Subscribers
        # State
        self.subscriber = self.create_subscription(
            Int32, 'state', self.response_callback, 10)

        # Image
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.br = CvBridge()


    # Publishers
        # States
        self.pub_response = self.create_publisher(Int32, 'response', 10)

        # Turtlebot
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # Image
        self.pub_img = self.create_publisher(Image, 'view', 10)

        # Main logic callback
        self.timer = self.create_timer(0.1, self.timer_callback)


# Callbacks
    def image_callback(self, msg):
        self.latest_frame = self.br.imgmsg_to_cv2(msg)


    def response_callback(self, msg):
        response = Int32()
        if msg.data == 1:
            self.block = False
            response.data = 11
        else:
            self.block = True
            response.data = 10
        self.pub_response.publish(response)


    # Main logic
    def timer_callback(self):
        if self.block or self.latest_frame is None:
            return

        # Get the actual width of the frame
        height, width, _ = self.latest_frame.shape
        point_list = []

        for x in range(self.scan_start, width - 1):  # Loop from scan_start to width-1
            if self.latest_frame[self.line_scan_y, x][2] >= 225:  # Check the blue channel value
                point_list.append(x)

        if len(point_list) != 0:
            self.line_center = sum(point_list) / len(point_list)
        else:
            self.line_center = None

        msg = Twist()
        if self.line_center is None:
            msg.linear.x = self.min_velocity
            msg.angular.z = self.lost_turn_rate
        else:
            correction = self.ideal_center_x - self.line_center
            if correction < 0:
                correction *= 2  # Normalize the left and right error to be equal in total magnitude
            correction /= (self.max_correction / 2.0)  # Makes angular.z max out at 2 for both sides
            if correction > 0.9:
                correction = 0.9
            msg.angular.z = correction
            msg.linear.x = self.calculate_velocity(correction)
    
        self.get_logger().warn(f'Velocity: {msg.linear.x:.3f}, Correction: {msg.angular.z:.3f}')
        self.pub_vel.publish(msg)


# Helper functions
    # Make velocity inverse proportional to the error
    def calculate_velocity(self, correction):
        target_speed = self.max_velocity - abs(correction)
        if (target_speed < self.min_velocity):
            self.velocity = self.min_velocity
        elif (target_speed < self.velocity):
            self.velocity = target_speed
        else:
            self.velocity += self.velocity_step

        return min(self.velocity, self.max_velocity)

