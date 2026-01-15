import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# TODO: make sure the bot doesnt start without sensor data

class Master(Node):
    def __init__(self):
        super().__init__('master')
        self.get_logger().info('Hello from master')

    # Variables
        # Flags
        self.block = False
        self.last_state = None
        self.latest_response = [None] * 5
        
        # Sensor data
        self.range = None
        self.is_wall = False
        self.latest_frame = None
        self.is_front_laser_far = False
        self.is_front_laser_near = False
        self.is_front_laser_right = False

        # Image
        self.height = 1080
        self.width = 1940
        self.stepSize = 64
        self.sign_detection_threshold = 225
        self.height_threshhold = int(self.height / 2)
        self.width_threshhold = int(self.width / 3)


    # Subscribers
        # Responses
        self.subscription = self.create_subscription(
            Int32, '/response', self.response_callback, 10)

        # Image
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 3)
        self.br = CvBridge()

        # LIDAR
        self.subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 3)


    # Publishers
        # States
        self.pub_command = self.create_publisher(Int32, 'state', 3)

        # Turtlebot
        self.pub_robot = self.create_publisher(Twist, 'cmd_vel', 3)

        # Main logic callback
        self.timer = self.create_timer(0.15, self.timer_callback)


# Callbacks
    def image_callback(self, msg):
        self.latest_frame = self.br.imgmsg_to_cv2(msg)

    def laser_callback(self, msg):
        self.is_front_laser_far = min(msg.ranges[0:25]) < 0.6
        self.is_front_laser_near = min(msg.ranges[0:10]) < 0.4
        self.is_front_laser_right = min(msg.ranges[340:345]) < 0.7
        

    def response_callback(self, msg):
        source, response = divmod(msg.data, 10)
        self.latest_response[source] = response
        name = 'Someone'
        if source == 1:
            name = 'Linefollower'
        elif source == 2:
            name = 'Avoider'
        elif source == 3:
            name = 'Tunnelenterer'
        elif source == 4:
            name = 'Tunnelpasser'
        self.get_logger().info(f'{name} responded with {response}')


    # Main logic
    def timer_callback(self):
        if self.block:
            if self.latest_response[2] == 2 or self.latest_response[4] == 2:
                self.block = False
            if self.latest_response[3] == 2:
                self.setState(4)
            return

        # Check Wall
        if self.is_front_laser_far:
            # Check Sign
            if self.detect_sign():
                self.setState(3)
            else:
                if self.is_front_laser_near and not self.is_front_laser_right:
                    self.setState(2)
                else:
                    self.setState(1)
        else:
            self.setState(1)


    # Helper functions
    def detect_sign(self):
        if self.latest_frame is None:
            return
        
        # IMPORTANT: to read frame[y,x] to paint cv2(x,y)
        for x in range(0, self.width_threshhold, self.stepSize):
            for y in range(0, self.height_threshhold, self.stepSize):
                if (self.latest_frame[y, x][0] >= self.sign_detection_threshold):
                    return True
        return False


    def setState(self, state):        
        if not self.wrongState(state):
            return

        msg = Int32()
        msg.data = state
      
        if state == 2 or state == 3 or state == 4:
            self.block = True

        if self.last_state != state:
            stop = Twist()
            self.pub_robot.publish(stop)
            self.get_logger().warn(f'Velocity: {stop.linear.x}, Correction: {stop.angular.z}')
            self.last_state = state

        self.get_logger().info(f'Publishing state: {msg.data}')
        self.pub_command.publish(msg)


    def wrongState(self, state):
        for x in range(1,5):
            if x == state:
                if self.latest_response[x] != 1:
                    return True
            else:
                if self.latest_response[x] != 0:
                    return True
        return False
    