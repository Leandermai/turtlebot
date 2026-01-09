import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Secret_tunnel(Node):
    def __init__(self):
        super().__init__('secret_tunnel')
        # Get camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            2)
        self.br = CvBridge()
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("camera", 640, 480)
    
        # Get Laser data
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Tell robot to drive
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.timer = self.create_timer(0.4, self.timer_callback)
    
    # Global variables ----- ----- -----
    scale_factor = 0.3   # Downscale image size
    height_threshhold = int(1080 * scale_factor / 2)
    width_threshhold = int(1940 * scale_factor / 3)
    found_tunel_entrance = False    # The flag that tells the bot when he has found he tunnel and has to navigate into it
    in_tunel = False    # Flag for when inside the tunnel

    # Save the latest image
    def image_callback(self, data):
        self.latest_frame = cv2.resize(self.br.imgmsg_to_cv2(data), (0,0), fx=self.scale_factor, fy=self.scale_factor)
        self.new_image = True

    # Save the current laser data
    def laser_callback(self, msg):
        self.ranges = msg.ranges[20:60] # Distance range: 0.12 - 3.5
        self.right = msg.ranges[270]

    # Drive into the tunnel ----- ----- -----
    def timer_callback(self):
        current_frame = self.latest_frame

        # If you are not at the entrance keep looking for it
        if not self.found_tunel_entrance:

            # Check wall
            distance = min(self.ranges)
            if (distance < 0.4):
                checkWall = True
                self.get_logger().info(f'Checking wall at: {distance}')
            else:
                checkWall = False

            # If there is a wall, is there a sign on the wall?
            if checkWall:
                # IMPORTANT: to read frame[y,x] to paint cv2(x,y)
                stepSize = 16   # maybe calculate based on threshhold or factor
                for x in range(0, self.width_threshhold, stepSize):
                    for y in range(0, self.height_threshhold, stepSize):
                        if (current_frame[y, x][0] >= 225):
                            cv2.circle(current_frame, (x,y), 3, (10, 10, 250), -1)
                            self.found_tunel_entrance = True   # TODO: check if there needs to be confirmation to avoid false positive

            # Paint whether check sign is detected
            if checkWall:
                cv2.circle(current_frame, (self.width_threshhold+4, 10), 3, (25, 25, 250), -1)

            # Act
            msg = Twist()
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info(f'Forward: {msg.linear.x}, Rotation: {msg.angular.z:.3f}')

        # Drive into the tunel, for now without alignment because the robot should already be aligned by the line follower
        else:
            if not self.in_tunel:
                msg = Twist()
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.get_logger().info(f'GO!')

                if (self.right < 1.0):
                    self.in_tunel = True

            else:
                msg = Twist()
                msg.linear.x = 0.0
                self.publisher.publish(msg)
                self.get_logger().info(f'Secret Tunel!')

        # Display
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    secret_tunnel = Secret_tunnel()
    rclpy.spin(secret_tunnel)
    secret_tunnel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
