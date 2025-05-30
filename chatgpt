from enum import Enum
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath import SE3
from roboticstoolbox import ERobot, DHRobot, RevoluteDH
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation as R
import numpy as np
from std_msgs.msg import Bool
from gazebo_ros_link_attacher.srv import Attach


class State(Enum):
    WAITING_FOR_ARUCO = 1
    MOVE_TO_POSE_1 = 2
    MOVE_TO_POSE_2 = 11
    GRIP_OBJECT = 3
    MOVE_UP_COLA = 12
    MOVE_COLA_TO_FINISH = 13
    RELEASE_OBJECT = 4
    MOVE_TO_HOME = 5
    INITIALIZE_GRIPPER_2 = 6
    MOVE_TO_POSE_3 = 7
    MOVE_TO_POSE_4 = 14
    GRIP_OBJECT_2 = 8
    MOVE_PRINGLES_TO_FINISH = 15
    RELEASE_OBJECT_2 = 9
    MOVE_TO_HOME_2 = 10
    MOVE_UP_PRINGLES = 17
    INITIALIZE_GRIPPER = 16


class KinematicPlanner(Node):
    def __init__(self):
        super().__init__('kinematic_planner')
        self.get_logger().info("Inizializzazione Kinematic Planner...")

        self.current_state = State.WAITING_FOR_ARUCO

        # Carica modello URDF
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info("URDF caricato correttamente!")

        # Publishers per i controller (arm e torso)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.completed_command_topic = self.create_publisher(Int32, '/completed_command_topic', 10)


        self.aruco_pose_1 = None
        self.aruco_pose_2 = None
        self.aruco_pose_3 = None
        self.aruco_pose_4 = None


        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.create_subscription(Int32,'/command_topic', self.command_callback, 10)
        self.aruco_sub_1 = self.create_subscription(PoseStamped, '/aruco_pose_1', self.aruco_pose_1_callback, 10)
        self.aruco_sub_2=self.create_subscription(PoseStamped, '/aruco_pose_2', self.aruco_pose_2_callback, 10)
        self.aruco_sub_3=self.create_subscription(PoseStamped, '/aruco_pose_3', self.aruco_pose_3_callback, 10)
        self.aruco_sub_4=self.create_subscription(PoseStamped, '/aruco_pose_4', self.aruco_pose_4_callback, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # Stato attuale dei giunti (vettore di 8: torso + 7 arm)
        self.current_joint_state = None  # np.array([torso, arm1, ..., arm7])
        self.target_pose = None  # PoseStamped

        self.attach_client = self.create_client(Attach, 'attach')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aspettando che il servizio /attach sia disponibile...')

        self.detach_client = self.create_client(Attach, 'detach')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aspettando che il servizio /detach sia disponibile...')
        


    def command_callback(self, msg):
        self.get_logger().info("Ricevuto comando")
        if msg.data == State.MOVE_TO_POSE_1.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_POSE_1")
            self.current_state = State.MOVE_TO_POSE_1
            self.calculate_pose_with_offset_state(2, offset_x=0, offset_y=-0.05, offset_z=0)
        elif msg.data == State.MOVE_TO_POSE_2.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_POSE_2")
            self.current_state = State.MOVE_TO_POSE_2
            self.calculate_pose_with_offset_state(2, offset_x=0, offset_y=-0.05, offset_z=-0.15)
        elif msg.data == State.GRIP_OBJECT.value:
            self.get_logger().info("Comando ricevuto: GRIP_OBJECT")
            self.current_state = State.GRIP_OBJECT
            self.close_gripper()
        elif msg.data == State.MOVE_UP_COLA.value:
            self.get_logger().info("Comando ricevuto: MOVE_UP_COLA")
            self.current_state = State.MOVE_UP_COLA
            self.calculate_pose_with_offset_state(2, offset_x=0, offset_y=-0.05, offset_z=0.1)
        elif msg.data == State.MOVE_COLA_TO_FINISH.value:
            self.get_logger().info("Comando ricevuto: MOVE_COLA_TO_FINISH")
            self.current_state = State.MOVE_COLA_TO_FINISH
            self.calculate_pose_with_offset_state(3, offset_x=0.0, offset_y=0, offset_z=0.01)
        elif msg.data == State.RELEASE_OBJECT.value:
            self.get_logger().info("Comando ricevuto: RELEASE_OBJECT")
            self.current_state = State.RELEASE_OBJECT
            self.open_gripper()
        elif msg.data == State.MOVE_TO_HOME.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_HOME")
            self.current_state = State.MOVE_TO_HOME
            self.move_to_home()
        elif msg.data == State.MOVE_TO_POSE_3.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_POSE_3")
            self.current_state = State.MOVE_TO_POSE_3
            self.calculate_pose_with_offset_state(1, offset_x=-0.07, offset_y=0.10, offset_z=-0.12)
        elif msg.data == State.MOVE_TO_POSE_4.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_POSE_4")
            self.current_state = State.MOVE_TO_POSE_4
            self.calculate_pose_with_offset_state(1, offset_x=-0.06, offset_y=0.10, offset_z=-0.20)
        elif msg.data == State.GRIP_OBJECT_2.value:
            self.get_logger().info("Comando ricevuto: GRIP_OBJECT_2")
            self.current_state = State.GRIP_OBJECT_2
            self.close_gripper()
        elif msg.data == State.MOVE_UP_PRINGLES.value:
            self.get_logger().info("Comando ricevuto: MOVE_UP_PRINGLES")
            self.current_state = State.MOVE_UP_PRINGLES
            self.calculate_pose_with_offset_state(1, offset_x=-0.06, offset_y=0.15, offset_z=0.1)
        elif msg.data == State.MOVE_PRINGLES_TO_FINISH.value:
            self.get_logger().info("Comando ricevuto: MOVE_PRINGLES_TO_FINISH")
            self.current_state = State.MOVE_PRINGLES_TO_FINISH
            self.calculate_pose_with_offset_state(4, offset_x=0, offset_y=0.0, offset_z=0.13)
        elif msg.data == State.RELEASE_OBJECT_2.value:
            self.get_logger().info("Comando ricevuto: RELEASE_OBJECT_2")
            self.current_state = State.RELEASE_OBJECT_2
            self.open_gripper()
        elif msg.data == State.MOVE_TO_HOME_2.value:
            self.get_logger().info("Comando ricevuto: MOVE_TO_HOME_2")
            self.current_state = State.MOVE_TO_HOME_2
            self.move_to_home()


    def aruco_pose_1_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_1")
        self.aruco_pose_1 = msg


    def aruco_pose_2_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_2")
        self.aruco_pose_2 = msg
        command_completed = Int32()
        if self.current_state == State.WAITING_FOR_ARUCO:
            command_completed.data = State.WAITING_FOR_ARUCO.value
            self.completed_command_topic.publish(command_completed)

    def aruco_pose_3_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_3")
        self.aruco_pose_3 = msg

    def aruco_pose_4_callback(self, msg):
        self.get_logger().info("Ricevuto aruco_pose_4")
        self.aruco_pose_4 = msg

    def joint_states_callback(self, msg):
        if self.current_joint_state is not None:
        # Hai già la configurazione iniziale, ignora i nuovi messaggi
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
        while self.current_joint_state is None:
            time.sleep(0.1)  # Attendi che il joint state sia inizializzato
        self.plan_and_publish_trajectory()


    def plan_and_publish_trajectory(self):
        self.get_logger().info("Pianificazione e pubblicazione della traiettoria...")

        if self.current_joint_state is None or self.target_pose is None:
            self.get_logger().warn("Joint state o target pose non ancora ricevuti.")
            return

        q0 = self.current_joint_state.astype(float)  # [torso, arm1, ..., arm7]

        # 1. Calcola T0 (configurazione attuale)
        try:
            T0 = self.robot.fkine(q0)
            self.get_logger().info(f"FKine calcolata: {T0}")
        except Exception as e:
            self.get_logger().error(f"Errore nel calcolo della fkine: {e}")
            return

        # 2. Estrai posizione e orientamento target
        pos = self.target_pose.pose.position
        ori = self.target_pose.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])  
        quaternion = [ori.x, ori.y, ori.z, ori.w]

        try:
            rotation = R.from_quat(quaternion).as_matrix()
            T_marker = SE3.Rt(rotation, position)
            if self.current_state == State.MOVE_COLA_TO_FINISH or self.current_state == State.MOVE_PRINGLES_TO_FINISH:
                T_rot = SE3.Rx(np.pi/2) * SE3.Rz(np.pi)
            else:
                T_rot = SE3.Ry(np.pi/2) * SE3.Rz(-(np.pi/2))
            TF = T_marker * T_rot
            self.get_logger().info(f"TF calcolata: {TF}")
        except Exception as e:
            self.get_logger().error(f"Errore nella costruzione della trasformazione finale TF: {e}")
            return

        N = 20  
        try:
            trajectory = rtb.ctraj(T0, TF, N)
        except Exception as e:
            self.get_logger().error(f"Errore nella generazione della traiettoria: {e}")
            return

        q_traj = []
        q_curr = q0
        '''try:
            qf = self.robot.ik_LM(TF, q0=q_curr)
            self.get_logger().info(f"Posizione finale calcolata: {qf}")
        except Exception as e:
            self.get_logger().error(f"Errore IK finale: {e}")
            return'''

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
        time.sleep(10)
        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.get_logger().info(f"Comando completato: {command_completed.data}")
        self.completed_command_topic.publish(command_completed)




    def calculate_pose_with_offset_state(self, target, offset_x, offset_y, offset_z):
        self.get_logger().info("Calcolo la posa con offset...")
        if target == 1:
            base_pose = self.aruco_pose_1.pose
            header = self.aruco_pose_1.header
        elif target == 2:
            base_pose = self.aruco_pose_2.pose
            header = self.aruco_pose_2.header
        elif target == 3:
            base_pose = self.aruco_pose_3.pose
            header = self.aruco_pose_3.header
        elif target == 4:
            base_pose = self.aruco_pose_4.pose
            header = self.aruco_pose_4.header

        # Creazione della nuova posa con offset
        target_pose_out = PoseStamped()
        target_pose_out.header.stamp = self.get_clock().now().to_msg()
        target_pose_out.header.frame_id = header.frame_id

        # Copia la posizione e aggiunge offset in z
        target_pose_out.pose.position.x = base_pose.position.x + offset_x
        target_pose_out.pose.position.y = base_pose.position.y + offset_y
        target_pose_out.pose.position.z = base_pose.position.z + offset_z

        # Copia l'orientamento
        target_pose_out.pose.orientation = base_pose.orientation

        # Ciclo per il controllo
        self.pose_pub.publish(target_pose_out)
        self.get_logger().info(" Posa target pubblicata su /target_pose")

        return "go_to_pose"
    
    def close_gripper(self):
        self.get_logger().info("Chiusura gripper in corso...")

        # Movimento gripper
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.041, 0.041]
        point.time_from_start.sec = 2
        traj.points.append(point)
        time.sleep(0.5)
        self.gripper_pub.publish(traj)

        time.sleep(3)
    
        if self.current_state == State.GRIP_OBJECT:
            object_name = "cocacola"
        elif self.current_state == State.GRIP_OBJECT_2:
            object_name = "pringles"
        
        
        os.system("ros2 service call /attach gazebo_ros_link_attacher/srv/Attach \"{model_name_1: tiago, link_name_1: gripper_left_finger_link, model_name_2: "+ str(object_name)+", link_name_2: link}\"")
        self.get_logger().info("Invio richiesta al servizio /attach...")
        time.sleep(1)
        
        
        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.completed_command_topic.publish(command_completed)

    def open_gripper(self):
        self.get_logger().info("Apertura a gripper in corso...")

        # Movimento gripper
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start.sec = 2
        traj.points.append(point)
        time.sleep(0.5)
        self.gripper_pub.publish(traj)

        time.sleep(3)

        object_name = None

        if self.current_state == State.RELEASE_OBJECT:
            object_name = "cocacola"
        elif self.current_state == State.RELEASE_OBJECT_2:
            object_name = "pringles"
        
        
        os.system("ros2 service call /detach gazebo_ros_link_attacher/srv/Attach \"{model_name_1: tiago, link_name_1: gripper_left_finger_link, model_name_2: "+ str(object_name)+", link_name_2: link}\"")
        self.get_logger().info("Invio richiesta al servizio /detach...")
        time.sleep(1)


        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.completed_command_topic.publish(command_completed)


    def move_to_home(self):
        self.get_logger().info("Ritorno alla posa finale predefinita...")

        # Prepara trajectory per torso
        traj_torso = JointTrajectory()
        traj_torso.joint_names = ['torso_lift_joint']

        point_torso = JointTrajectoryPoint()
        point_torso.positions = [0.35]
        point_torso.time_from_start.sec = 3
        point_torso.time_from_start.nanosec = 0

        traj_torso.points.append(point_torso)

        # Prepara trajectory per braccio
        traj_arm = JointTrajectory()
        traj_arm.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        point_arm = JointTrajectoryPoint()
        point_arm.positions = [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05]
        point_arm.time_from_start.sec = 3
        point_arm.time_from_start.nanosec = 0

        traj_arm.points.append(point_arm)

        # Pubblica
        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)

        self.get_logger().info("Traiettoria di ritorno pubblicata su torso e braccio.")
        time.sleep(10)

        command_completed = Int32()
        command_completed.data = self.current_state.value
        self.get_logger().info(f"Comando completato: {command_completed.data}")
        self.completed_command_topic.publish(command_completed)



def main(args=None):
    rclpy.init(args=args)
    node = KinematicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
