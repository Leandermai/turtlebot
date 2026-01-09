from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy

class Line_follower(Node):
    screen_center = 960
    line_center = 0
    ideal_center = 1440 # this value is listed in the cv_color_detect 
    max_correction = 960    # this value is in the cv_color_detect
    velocity = 0.1
    def __init__(self):
        super().__init__('laser_controller')

        self.subscription = self.create_subscription(
            Vector3,
            'line_center',
            self.image_callback,
            2)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.25, self.publish_commands)

    def image_callback(self, msg):
        self.line_center = msg.x

    def publish_commands(self):
        msg = Twist()
        correction = self.ideal_center - self.line_center
        if (correction < 0):
            correction *= 2 # normalize the left and right error to be equal in total magnitude
        correction /= (self.max_correction / 2.0)   # makes the angular.z max out at 2 for both sides
        if (correction > 0.9):
            correction = 0.9
        msg.angular.z = correction
        msg.linear.x = self.calculate_velocity(correction)
        self.get_logger().info(f'Velocity: {msg.linear.x:.3f}, Correction: {msg.angular.z:.3f}')
        self.publisher.publish(msg)

    def calculate_velocity(self, correction, max_vel = 0.6):
        target_speed = max_vel - abs(correction)
        if (target_speed < 0.1): self.velocity= 0.1
        elif (target_speed < self.velocity):
            self.velocity = target_speed
        else:
            self.velocity += 0.025
        return self.velocity

def main(args=None):
    rclpy.init(args=args)
    line_follower = Line_follower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# TODO: figure out max sensor read speed