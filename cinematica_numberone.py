import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from spatialmath import SE3
from roboticstoolbox import ERobot

class KinematicPlanner(Node):
    def __init__(self):
        super().__init__('kinematic_planner')

        # Carica URDF (modifica path se necessario)
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info("Robot caricato da URDF!")

        # Subscribe ai topic della macchina a stati
        self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)
        self.create_subscription(String, '/command_topic', self.command_callback, 10)

        # Action client per arm, torso, gripper
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.torso_client = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Salva ultimo comando e pose ricevuti
        self.current_pose = None
        self.current_task = None

    def pose_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info("Pose ricevuta.")
        self.try_execute()

    def command_callback(self, msg):
        self.current_task = msg.data
        self.get_logger().info(f"Task ricevuto: {self.current_task}")
        self.try_execute()

    def try_execute(self):
        if self.current_pose is not None and self.current_task is not None:
            if self.current_task in ['grasp', 'place', 'move_to_grasp']:
                self.plan_and_send_arm_trajectory()
            elif self.current_task in ['open_gripper', 'close_gripper']:
                self.send_gripper_command()
            else:
                self.get_logger().warn(f"Task sconosciuto: {self.current_task}")

    def plan_and_send_arm_trajectory(self):
        pose = self.current_pose.pose
        target_se3 = SE3(pose.position.x, pose.position.y, pose.position.z)
        # Usa la configurazione centrale come esempio
        q0 = np.zeros(self.robot.n)  # Cambia se vuoi una posizione diversa
        N = 20  # passi della traiettoria

        # Interpolazione cart. + IK
        Ts = self.robot.fkine(q0).interp(target_se3, N)
        q_traj = []
        q_curr = q0
        for T in Ts:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warn("IK non trovata per uno step.")
                break
        if not q_traj:
            self.get_logger().error("Nessuna soluzione trovata! Abbandono.")
            return

        # Torso: prendi il valore del torso dalla soluzione, il resto per arm
        torso_traj = []
        arm_traj = []
        for q in q_traj:
            if len(q) == 8:
                torso_traj.append([q[0]])
                arm_traj.append(q[1:])  # [1:] = arm_1 ... arm_7
            else:
                torso_traj.append([0.0])
                arm_traj.append(q)

        # -- Prepara messaggio per TORSO --
        torso_msg = JointTrajectory()
        torso_msg.joint_names = ['torso_lift_joint']
        for i, tq in enumerate(torso_traj):
            point = JointTrajectoryPoint()
            point.positions = tq
            point.time_from_start.sec = int(i * 0.1)
            torso_msg.points.append(point)

        # -- Prepara messaggio per ARM --
        arm_msg = JointTrajectory()
        arm_msg.joint_names = [f'arm_{i+1}_joint' for i in range(7)]
        for i, aq in enumerate(arm_traj):
            point = JointTrajectoryPoint()
            point.positions = aq.tolist()
            point.time_from_start.sec = int(i * 0.1)
            arm_msg.points.append(point)

        # -- Invia tramite Action Client --
        # TORSO
        torso_goal = FollowJointTrajectory.Goal()
        torso_goal.trajectory = torso_msg
        self.torso_client.wait_for_server()
        self.torso_client.send_goal_async(torso_goal)
        self.get_logger().info("Traiettoria torso inviata.")

        # ARM
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory = arm_msg
        self.arm_client.wait_for_server()
        self.arm_client.send_goal_async(arm_goal)
        self.get_logger().info("Traiettoria braccio inviata.")

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
    node = KinematicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
