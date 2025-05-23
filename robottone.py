from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath import SE3
from roboticstoolbox import ERobot
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation as R
import time

class State(Enum):
    WAITING_FOR_ARUCO = 1
    MOVE_TO_POSE_1 = 2
    MOVE_DOWN = 11
    GRIP_OBJECT = 3
    RELEASE_OBJECT = 4
    MOVE_TO_HOME = 5
    INITIALIZE_GRIPPER_2 = 6
    MOVE_TO_POSE_2 = 7
    GRIP_OBJECT_2 = 8
    RELEASE_OBJECT_2 = 9
    MOVE_TO_HOME_2 = 10

class KinematicPlanner(Node):
    def __init__(self):
        super().__init__('kinematic_planner')
        self.get_logger().info("Inizializzazione Kinematic Planner...")

        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info("URDF caricato correttamente!")

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.completed_command_topic = self.create_publisher(Int32, '/completed_command_topic', 10)

        self.aruco_pose_1 = None
        self.aruco_pose_2 = None
        self.aruco_pose_3 = None
        self.aruco_pose_4 = None

        self.current_joint_state = None  # np.array([torso, arm1, ..., arm7])
        self.target_pose = None  # PoseStamped

        # Variabile per salvare l'orientazione raggiunta dall'end-effector
        self.last_reached_orientation = None

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.create_subscription(Int32, '/command_topic', self.command_callback, 10)
        self.aruco_sub_1 = self.create_subscription(PoseStamped, '/aruco_pose_1', self.aruco_pose_1_callback, 10)
        self.aruco_sub_2 = self.create_subscription(PoseStamped, '/aruco_pose_2', self.aruco_pose_2_callback, 10)
        self.aruco_sub_3 = self.create_subscription(PoseStamped, '/aruco_pose_3', self.aruco_pose_3_callback, 10)
        self.aruco_sub_4 = self.create_subscription(PoseStamped, '/aruco_pose_4', self.aruco_pose_4_callback, 10)

    def command_callback(self, msg):
        self.get_logger().info("Ricevuto comando")

        if msg.data == State.MOVE_TO_POSE_1.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_POSE_1")
            # Primo movimento: usa orientamento del marker
            self.calculate_pose_with_offset_state(2, offset_x=0.01, offset_y=-0.1, offset_z=0.0, use_last_orientation=False)
            time.sleep(10)
        elif msg.data == State.MOVE_DOWN.value:
            self.get_logger().info("Comando ricevuto: MOVE_DOWN")
            # Usa la rotazione raggiunta precedentemente (Nessuna nuova rotazione!)
            self.calculate_pose_with_offset_state(2, offset_x=0.0, offset_y=0.0, offset_z=-0.15, use_last_orientation=True)
        else:
            self.get_logger().warning("Comando sconosciuto")

    def aruco_pose_1_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_1")
        self.aruco_pose_1 = msg

    def aruco_pose_2_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_2")
        self.aruco_pose_2 = msg
        command_completed = Int32()
        command_completed.data = State.WAITING_FOR_ARUCO.value
        self.completed_command_topic.publish(command_completed)

    def aruco_pose_3_callback(self, msg):
        self.aruco_pose_3 = msg

    def aruco_pose_4_callback(self, msg):
        self.aruco_pose_4 = msg

    def joint_states_callback(self, msg):
        if self.current_joint_state is not None:
            return
        joint_names = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        joint_pos = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_pos.append(msg.position[idx])
            else:
                joint_pos.append(0.0)
        self.current_joint_state = np.array(joint_pos)
        self.get_logger().info(f"[JOINT INIT] Salvata configurazione iniziale: {joint_pos}")

    def target_pose_callback(self, msg):
        self.get_logger().info("Funzione target_pose_callback chiamata!")
        self.target_pose = msg
        self.get_logger().info(f" Ricevuta target_pose: {msg.pose.position}")
        # Salva l'orientamento effettivamente raggiunto (ogni volta che ricevi la target_pose)
        self.last_reached_orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        self.get_logger().info("Pianificazione e pubblicazione della traiettoria...")

        if self.current_joint_state is None or self.target_pose is None:
            self.get_logger().warn("Joint state o target pose non ancora ricevuti.")
            return

        q0 = self.current_joint_state.astype(float)  # [torso, arm1, ..., arm7]

        try:
            T0 = self.robot.fkine(q0)
            self.get_logger().info(f"FKine calcolata: {T0}")
        except Exception as e:
            self.get_logger().error(f"Errore nel calcolo della fkine: {e}")
            return

        pos = self.target_pose.pose.position
        ori = self.target_pose.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        quaternion = [ori.x, ori.y, ori.z, ori.w]

        try:
            rotation = R.from_quat(quaternion).as_matrix()
            pi = np.pi
            T_marker = SE3.Rt(rotation, position)
            T_rot = SE3.Ry(pi/2) * SE3.Rz(-pi/2)
            TF = T_marker * T_rot
            self.get_logger().info(f"TF calcolata: {TF}")
        except Exception as e:
            self.get_logger().error(f"Errore nella costruzione della trasformazione finale TF: {e}")
            return

        N = 10
        try:
            trajectory = rtb.ctraj(T0, TF, N)
        except Exception as e:
            self.get_logger().error(f"Errore nella generazione della traiettoria: {e}")
            return

        q_traj = []
        q_curr = q0

        for i, T in enumerate(trajectory):
            sol = self.robot.ik_LM(T, q0=q_curr)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warning(f"IK fallita al punto {i}. Riutilizzo configurazione precedente")
                q_traj.append(q_curr.copy())

        q_traj = np.array(q_traj)
        self.get_logger().info(f"Traiettoria generata con successo: {q_traj}")

        # Prepara e pubblica SOLO l'ultimo punto della traiettoria
        traj_arm = JointTrajectory()
        traj_torso = JointTrajectory()
        traj_arm.joint_names = [f'arm_{i+1}_joint' for i in range(7)]
        traj_torso.joint_names = ['torso_lift_joint']

        q = q_traj[-1]
        point_arm = JointTrajectoryPoint()
        point_torso = JointTrajectoryPoint()
        point_arm.positions = [float(p) for p in q[1:]]
        point_torso.positions = [float(q[0])]

        point_arm.time_from_start.sec = 5
        point_arm.time_from_start.nanosec = 0
        point_torso.time_from_start.sec = 5
        point_torso.time_from_start.nanosec = 0

        traj_arm.points.append(point_arm)
        traj_torso.points.append(point_torso)

        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)
        self.get_logger().info("ultimo punto della traiettoria è stato pubblicato")

    def calculate_pose_with_offset_state(self, target, offset_x, offset_y, offset_z, use_last_orientation=False):
        self.get_logger().info("Calcolo la posa con offset...")
        if target == 1:
            base_pose = self.aruco_pose_1.pose
            header = self.aruco_pose_1.header
        else:
            base_pose = self.aruco_pose_2.pose
            header = self.aruco_pose_2.header

        target_pose_out = PoseStamped()
        target_pose_out.header.stamp = self.get_clock().now().to_msg()
        target_pose_out.header.frame_id = header.frame_id
        target_pose_out.pose.position.x = base_pose.position.x + offset_x
        target_pose_out.pose.position.y = base_pose.position.y + offset_y
        target_pose_out.pose.position.z = base_pose.position.z + offset_z

        # Mantieni orientamento raggiunto, se richiesto
        if use_last_orientation and self.last_reached_orientation is not None:
            target_pose_out.pose.orientation.x = self.last_reached_orientation[0]
            target_pose_out.pose.orientation.y = self.last_reached_orientation[1]
            target_pose_out.pose.orientation.z = self.last_reached_orientation[2]
            target_pose_out.pose.orientation.w = self.last_reached_orientation[3]
        else:
            target_pose_out.pose.orientation = base_pose.orientation

        self.pose_pub.publish(target_pose_out)
        self.get_logger().info(" Posa target pubblicata su /target_pose")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
