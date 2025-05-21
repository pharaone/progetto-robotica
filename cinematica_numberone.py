import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from spatialmath import SE3
from roboticstoolbox import ERobot

class KinematicPlanner(Node):
    def __init__(self):
        super().__init__('kinematic_planner')

        # Carica URDF del robot
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info('URDF caricato correttamente!')

        # Publisher per arm e torso (SOLO publisher, NO ActionClient)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
        # Publisher opzionale per il gripper (puoi usare anche /gripper_controller/joint_trajectory)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # Subscriber
        self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)
        self.create_subscription(String, '/command_topic', self.task_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Stato interno
        self.current_pose = None
        self.current_task = None
        self.current_joint_state = None  # Verrà un vettore numpy con [torso, arm_1...7]

    def joint_states_callback(self, msg):
        # Prende SOLO i giunti che ti interessano (torso_lift_joint + arm_1..7)
        joint_order = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        joint_pos = []
        for name in joint_order:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_pos.append(msg.position[idx])
            else:
                joint_pos.append(0.0)
        self.current_joint_state = np.array(joint_pos)

    def pose_callback(self, msg):
        self.current_pose = msg
        self.try_execute()

    def task_callback(self, msg):
        self.current_task = msg.data
        self.try_execute()

    def try_execute(self):
        if self.current_pose is not None and self.current_task is not None and self.current_joint_state is not None:
            if self.current_task in ['grasp', 'place', 'move_to_grasp']:
                self.plan_and_send_arm_trajectory()
            elif self.current_task in ['open_gripper', 'close_gripper']:
                self.send_gripper_command()

    def plan_and_send_arm_trajectory(self):
        # Usa la posizione attuale come q0 (torso + 7 arm)
        q0 = self.current_joint_state.copy()
        if len(q0) != 8:
            self.get_logger().error("Joint state errato. Mi aspetto 8 joint (1 torso + 7 arm).")
            return

        pose = self.current_pose.pose
        target_se3 = SE3(pose.position.x, pose.position.y, pose.position.z)
        N = 30
        Ts = self.robot.fkine(q0).interp(target_se3, N)
        q_traj = []
        q_curr = q0
        for T in Ts:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warn("IK non trovata per uno degli step.")
                break
        if not q_traj:
            self.get_logger().error("Nessuna soluzione IK trovata. Comando non inviato.")
            return

        # Costruisci messaggi separati per torso e arm
        torso_msg = JointTrajectory()
        torso_msg.joint_names = ['torso_lift_joint']
        arm_msg = JointTrajectory()
        arm_msg.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        for i, q in enumerate(q_traj):
            # q[0]: torso, q[1:]: arm
            pt_torso = JointTrajectoryPoint()
            pt_torso.positions = [q[0]]
            pt_torso.time_from_start.sec = int(i * 0.1)
            torso_msg.points.append(pt_torso)

            pt_arm = JointTrajectoryPoint()
            pt_arm.positions = q[1:].tolist()
            pt_arm.time_from_start.sec = int(i * 0.1)
            arm_msg.points.append(pt_arm)

        # PUBBLICA i messaggi sui due topic separati
        self.torso_pub.publish(torso_msg)
        self.arm_pub.publish(arm_msg)
        self.get_logger().info("Inviata traiettoria a torso e arm via publisher.")

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
        self.gripper_pub.publish(traj_msg)
        self.get_logger().info("Comando gripper inviato.")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
