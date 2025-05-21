import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

from spatialmath import SE3
from roboticstoolbox import ERobot


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info(str(self.robot))

        self.current_pose = None
        self.current_task = None
        self.current_joint_state = None
        self.q0 = None  # Valore iniziale per il joint state [torso + 7 arm joints]

        self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)
        self.get_logger().info("📡 Subscription a /target_pose creata!")

        self.create_subscription(String, '/command_topic', self.task_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Publisher per JointTrajectory
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def joint_states_callback(self, msg: JointState):
        self.get_logger().info("Ricevuto JointState")
        self.get_logger().info(f"Joint names: {msg.name}")
        self.get_logger().info(f"Joint positions: {msg.position}")

        joint_order = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        self.get_logger().info(f"Ordinamento atteso: {joint_order}")

        joint_pos = []
        for name in joint_order:
            if name in msg.name:
                idx = msg.name.index(name)
                pos = msg.position[idx]
                self.get_logger().info(f"Trovato {name} all'indice {idx} con posizione {pos}")
                joint_pos.append(pos)
            else:
                self.get_logger().warn(f"Joint {name} non trovato in msg.name! Uso posizione 0.0")
                joint_pos.append(0.0)

        self.current_joint_state = np.array(joint_pos)
        self.get_logger().info(f"Current joint state array: {self.current_joint_state}")

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        self.get_logger().info("Ricevuto target pose")
        self.try_execute()

    def task_callback(self, msg: String):
        self.current_task = msg.data
        self.get_logger().info(f"Ricevuto comando task: {self.current_task}")
        self.try_execute()

    def try_execute(self):
        if self.current_pose is not None and self.current_task is not None:
            self.get_logger().info(f"Eseguo task {self.current_task}")
            if self.current_task in ['grasp', 'place']:
                self.plan_and_send_arm_trajectory()
            elif self.current_task in ['open_gripper', 'close_gripper']:
                self.send_gripper_command()

    def plan_and_send_arm_trajectory(self):
        self.get_logger().info("Planning e invio traiettoria braccio e torso")

        pose = self.current_pose.pose
        target_se3 = SE3(pose.position.x, pose.position.y, pose.position.z)

        q0 = self.current_joint_state if self.current_joint_state is not None else self.q0

        N = 50
        Ts = self.robot.fkine(q0).interp(target_se3, N)
        q_traj = []
        q_curr = q0
        for T in Ts:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warn("IK non trovata per uno degli step. Interrompo la traiettoria.")
                break

        if not q_traj:
            self.get_logger().error("Nessuna soluzione IK trovata. Comando non inviato.")
            return

        torso_traj = JointTrajectory()
        torso_traj.joint_names = ['torso_lift_joint']

        arm_traj = JointTrajectory()
        arm_traj.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        for i, q in enumerate(q_traj):
            if len(q) == 8:
                q = q.tolist()
            else:
                self.get_logger().warn("Soluzione IK non di lunghezza 8! Skipping step.")
                continue

            torso_point = JointTrajectoryPoint()
            torso_point.positions = [q[0]]
            torso_point.time_from_start = Duration(sec=int(i * 0.1), nanosec=0)
            torso_traj.points.append(torso_point)

            arm_point = JointTrajectoryPoint()
            arm_point.positions = q[1:7]
            arm_point.time_from_start = Duration(sec=int(i * 0.1), nanosec=0)
            arm_traj.points.append(arm_point)

        self.torso_pub.publish(torso_traj)
        self.arm_pub.publish(arm_traj)
        self.get_logger().info("Traiettoria torso e braccio inviata")

    def send_gripper_command(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=1, nanosec=0)
        if self.current_task == 'close_gripper':
            point.positions = [0.0, 0.0]
        else:
            point.positions = [0.04, 0.04]
        traj_msg.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal_msg)
        self.get_logger().info("Comando gripper inviato")


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
