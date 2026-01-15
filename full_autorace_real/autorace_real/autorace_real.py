import rclpy
from rclpy.executors import MultiThreadedExecutor
from autorace_real.master import Master
from autorace_real.follow_line import FollowLine
from autorace_real.pass_tunnel import PassTunnel
from autorace_real.enter_tunnel import EnterTunnel
from autorace_real.avoid_obstacle import AvoidObstacle

def main(args=None):
    rclpy.init(args=args)

    master = Master()
    follow_line = FollowLine()
    pass_tunnel = PassTunnel()
    enter_tunnel = EnterTunnel()
    avoid_obstacle = AvoidObstacle()

    executor = MultiThreadedExecutor()
    executor.add_node(master)
    executor.add_node(follow_line)
    executor.add_node(pass_tunnel)
    executor.add_node(enter_tunnel)
    executor.add_node(avoid_obstacle)

    try:
        executor.spin()
    finally:
        master.destroy_node()
        follow_line.destroy_node()
        pass_tunnel.destroy_node()
        enter_tunnel.destroy_node()
        avoid_obstacle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
