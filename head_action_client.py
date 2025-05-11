#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration

class TiagoHeadActionClient(Node):

    def __init__(self):
        super().__init__('tiago_head_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, 'move_head')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        # 1. Movimento iniziale: solo abbassare la testa
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -0.57]  # centro e giù
        point1.time_from_start = Duration(sec=2)

        # 2. Movimento laterale da destra a sinistra mantenendo testa giù
        point2 = JointTrajectoryPoint()
        point2.positions = [0.217, -0.57]
        point2.time_from_start = Duration(sec=4)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, -0.57]
        point3.time_from_start = Duration(sec=6)

        point4 = JointTrajectoryPoint()
        point4.positions = [-0.217, -0.57]
        point4.time_from_start = Duration(sec=8)

        goal_msg.trajectory.points = [point1, point2, point3, point4]

        self._action_client.wait_for_server()
        self.get_logger().info('Invio sequenza movimento testa')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal respinto')
            return
        self.get_logger().info('Goal accettato')

def main(args=None):
    rclpy.init(args=args)
    client = TiagoHeadActionClient()
    client.send_goal()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()