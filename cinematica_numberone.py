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

        self.get_logger().info("=== Nodo KinematicPlanner: Inizializzazione ===")

        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.get_logger().info(f"Caricamento URDF da: {urdf_loc}")
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info("URDF caricato correttamente!")

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)

        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        self.current_joint_state = None
        self.target_pose = None

        self.get_logger().info("=== Nodo pronto, aspetto messaggi... ===")

    def joint_states_callback(self, msg):
        joint_names = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        joint_pos = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_pos.append(msg.position[idx])
            else:
                joint_pos.append(0.0)
        self.current_joint_state = np.array(joint_pos)
        self.get_logger().info(f"Ricevuto joint_states: {joint_pos}")

    def target_pose_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(f"Ricevuta target_pose: "
                               f"x={msg.pose.position.x:.3f}, "
                               f"y={msg.pose.position.y:.3f}, "
                               f"z={msg.pose.position.z:.3f}")
        self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        self.get_logger().info("Chiamata plan_and_publish_trajectory()")
        if self.current_joint_state is None:
            self.get_logger().warn("Joint state NON ancora ricevuti. Attendo...")
            return
        if self.target_pose is None:
            self.get_logger().warn("Target pose NON ancora ricevuta. Attendo...")
            return

        pos = self.target_pose.pose.position
        self.get_logger().info(f"Pianifico verso target SE3: ({pos.x}, {pos.y}, {pos.z})")
        target_se3 = SE3(pos.x, pos.y, pos.z)

        q0 = self.current_joint_state
        N = 40

        try:
            self.get_logger().info(f"Calcolo traiettoria cartesiana dal punto attuale ({q0.tolist()})...")
            Ts = self.robot.fkine(q0).interp(target_se3, N)
        except Exception as e:
            self.get_logger().error(f"Errore durante interp traiettoria: {e}")
            return

        q_traj = []
        q_curr = q0.copy()
        for i, T in enumerate(Ts):
            try:
                sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
            except Exception as e:
                self.get_logger().error(f"Errore IK allo step {i}: {e}")
                break
            if sol is not None and len(sol) > 0:
                q_curr = sol[0]
                q_traj.append(q_curr)
                self.get_logger().info(f"Step {i}: Soluzione IK trovata: {q_curr.tolist()}")
            else:
                self.get_logger().warn(f"IK non trovata allo step {i}! Interrompo.")
                break

        if not q_traj:
            self.get_logger().error("IK non trovata, nessuna traiettoria pubblicata.")
            return

        self.get_logger().info(f"Costruisco e pubblico Trajectory di {len(q_traj)} step...")

        torso_traj = JointTrajectory()
        torso_traj.joint_names = ['torso_lift_joint']
        arm_traj = JointTrajectory()
        arm_traj.joint_names = [f'arm_{i+1}_joint' for i in range(7)]

        for i, q in enumerate(q_traj):
            t = i * 0.1
            torso_point = JointTrajectoryPoint()
            torso_point.positions = [float(q[0])]
            torso_point.time_from_start.sec = int(t)
            arm_point = JointTrajectoryPoint()
            arm_point.positions = [float(x) for x in q[1:]]
            arm_point.time_from_start.sec = int(t)
            torso_traj.points.append(torso_point)
            arm_traj.points.append(arm_point)

        self.get_logger().info(f"Pubblico torso_traj su /torso_controller/joint_trajectory ({len(torso_traj.points)} punti)")
        self.torso_pub.publish(torso_traj)
        self.get_logger().info(f"Pubblico arm_traj su /arm_controller/joint_trajectory ({len(arm_traj.points)} punti)")
        self.arm_pub.publish(arm_traj)
        self.get_logger().info("Traiettorie pubblicate con successo!")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
