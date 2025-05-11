import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
import time

class TiagoHeadActionServer(Node):

    def __init__(self):
        super().__init__('tiago_head_action_server')
        self.publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'move_head',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Ricevuto goal per movimento testa')

        trajectory = goal_handle.request.trajectory
        self.publisher.publish(trajectory)
        self.get_logger().info('Traiettoria inviata al topic /head_controller/joint_trajectory')

        duration = trajectory.points[-1].time_from_start.sec + 1
        time.sleep(duration)

        goal_handle.succeed()
        self.get_logger().info('Movimento completato')
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    server = TiagoHeadActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()