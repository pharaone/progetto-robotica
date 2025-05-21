import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from spatialmath import SE3
from roboticstoolbox import ERobot

class KinematicPlanner(Node):
    def __init__(self):
        super().__init__('kinematic_planner')
        self.get_logger().info("Inizializzazione Kinematic Planner...")

        # Carica modello URDF
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info("URDF caricato correttamente!")

        # Publishers per i controller (arm e torso)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)

        # Subscriptions
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # Stato attuale dei giunti (vettore di 8: torso + 7 arm)
        self.current_joint_state = None  # np.array([torso, arm1, ..., arm7])
        self.target_pose = None  # PoseStamped

    def joint_states_callback(self, msg):
        # Estrae la configurazione attuale dei giunti rilevanti (ordine importante!)
        joint_names = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        joint_pos = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_pos.append(msg.position[idx])
            else:
                joint_pos.append(0.0)
        self.current_joint_state = np.array(joint_pos)
        self.get_logger().info(f"Lettura joint_states: {joint_pos}")

    def target_pose_callback(self, msg):
        self.get_logger().info("✅ Funzione target_pose_callback chiamata!")
        self.target_pose = msg
        self.get_logger().info(f"✅ Ricevuta target_pose: {msg.pose.position}")
        self.plan_and_publish_trajectory()


    def plan_and_publish_trajectory(self):
        # Check che abbiamo tutto
        if self.current_joint_state is None or self.target_pose is None:
            self.get_logger().warn("Joint state o target pose non ancora ricevuti.")
            return

        # Costruisci SE3 della pose target (solo posizione, orientamento ignorato)
        pos = self.target_pose.pose.position
        target_se3 = SE3(pos.x, pos.y, pos.z)

        q0 = self.current_joint_state  # Stato attuale
        N = 40

        # Pianifica la traiettoria cartesiana dal punto attuale al target
        Ts = self.robot.fkine(q0).interp(target_se3, N)

        q_traj = []
        q_curr = q0.copy()
        for T in Ts:
            sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
            else:
                self.get_logger().warn("IK fallita per uno step della traiettoria!")
                break

        if not q_traj:
            self.get_logger().error("IK non trovata, nessuna traiettoria pubblicata.")
            return

        # Prepara messaggi JointTrajectory per torso e arm SEPARATI!
        # --- TORSO ---
        torso_traj = JointTrajectory()
        torso_traj.joint_names = ['torso_lift_joint']
        # --- ARM ---
        arm_traj = JointTrajectory()
        arm_traj.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        for i, q in enumerate(q_traj):
            t = i * 0.1  # 0.1s per step
            # TORSO point
            torso_point = JointTrajectoryPoint()
            torso_point.positions = [float(q[0])]
            torso_point.time_from_start.sec = int(t)
            torso_traj.points.append(torso_point)
            # ARM point
            arm_point = JointTrajectoryPoint()
            arm_point.positions = [float(x) for x in q[1:]]
            arm_point.time_from_start.sec = int(t)
            arm_traj.points.append(arm_point)

        self.torso_pub.publish(torso_traj)
        self.arm_pub.publish(arm_traj)
        self.get_logger().info("Pubblicate JointTrajectory su torso e arm controller.")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()