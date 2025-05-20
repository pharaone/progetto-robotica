import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH, ERobot

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        # Carica il robot da URDF (modo 1)
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        print(self.robot)

        # Sottoscrizioni ai topic dalla macchina a stati
        self.current_pose = None
        self.current_task = None
        self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)
        self.create_subscription(String, '/command_topic', self.task_callback, 10)

        # Client per invio ai controller
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Limiti articolari
        self.joint_angle_ranges = np.array([
            [-150 * (np.pi / 180), 114 * (np.pi / 180)],
            [-67 * (np.pi / 180), 109 * (np.pi / 180)],
            [-150 * (np.pi / 180), 41 * (np.pi / 180)],
            [-92 * (np.pi / 180), 110 * (np.pi / 180)],
            [-150 * (np.pi / 180), 150 * (np.pi / 180)],
            [92 * (np.pi / 180), 113 * (np.pi / 180)],
            [-150 * (np.pi / 180), 150 * (np.pi / 180)]
        ])
        self.q0 = np.mean(self.joint_angle_ranges, axis=1)

    def pose_callback(self, msg):
        self.current_pose = msg
        self.try_execute()

    def task_callback(self, msg):
        self.current_task = msg.data
        self.try_execute()

    def try_execute(self):
        if self.current_pose is not None and self.current_task is not None:
            if self.current_task in ['grasp', 'place']:
                self.plan_and_send_arm_trajectory()
            elif self.current_task in ['open_gripper', 'close_gripper']:
                self.send_gripper_command()

    def plan_and_send_arm_trajectory(self):
        pose = self.current_pose.pose
        target_se3 = SE3(pose.position.x, pose.position.y, pose.position.z)

        q0 = self.q0  # usa q0 calcolato dai joint limits
        N = 50
        Ts = self.robot.fkine(q0).interp(target_se3, N)

        q_traj = []
        q_curr = q0
        for T in Ts:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        for i, q in enumerate(q_traj):
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            point.time_from_start.sec = int(i * 0.1)
            traj_msg.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.arm_client.wait_for_server()
        self.arm_client.send_goal_async(goal_msg)
        self.get_logger().info("Inviata traiettoria al braccio.")

    def send_gripper_command(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.time_from_start.sec = 1

        if self.current_task == 'close_gripper':
            point.positions = [0.0, 0.0]
        else:
            point.positions = [0.04, 0.04]

        traj_msg.points.append(point)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal_msg)
        self.get_logger().info("Comando gripper inviato.")

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
