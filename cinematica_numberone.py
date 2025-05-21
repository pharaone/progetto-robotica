import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from spatialmath import SE3
from roboticstoolbox import ERobot, ctraj

class KinematicTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('kinematic_traj_pub')

        # Carica modello dal tuo URDF
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info('[INIT] Robot URDF caricato')

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # Publishers
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)

        # Stato iniziale
        self.q0 = None
        self.target_pose = None
        self.traj_ready = False
        self.q_traj = None

        # Timer per pubblicare la posa target in loop
        self.create_timer(0.1, self.loop_target_pose)

    def joint_states_callback(self, msg):
        if self.q0 is not None:
            return  # Salva una volta sola!

        joint_names = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        q_init = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                q_init.append(msg.position[idx])
            else:
                q_init.append(0.0)
        self.q0 = np.array(q_init)
        self.get_logger().info(f"[JOINT INIT] q0 acquisito: {q_init}")

    def target_pose_callback(self, msg):
        if self.target_pose is None:
            self.target_pose = msg
            self.get_logger().info(f"[POSE INIT] Target pose salvata: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

    def loop_target_pose(self):
        if self.q0 is None or self.target_pose is None:
            return

        if not self.traj_ready:
            # Calcola traiettoria
            T0 = self.robot.fkine(self.q0)
            pose = self.target_pose.pose
            Tf = SE3(pose.position.x, pose.position.y, pose.position.z)
            Ts = ctraj(T0, Tf, 60)

            q_traj = []
            q_curr = self.q0
            for T in Ts:
                sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
                if sol is not None and len(sol) > 0:
                    q_curr = sol[0]
                q_traj.append(q_curr)

            self.q_traj = np.array(q_traj)
            self.traj_ready = True
            self.index = 0
            self.get_logger().info(f"[TRAJ READY] Traiettoria calcolata con {len(q_traj)} punti.")

        # Pubblica ciclicamente
        q = self.q_traj[self.index % len(self.q_traj)]

        torso_traj = JointTrajectory()
        torso_traj.joint_names = ['torso_lift_joint']
        torso_point = JointTrajectoryPoint()
        torso_point.positions = [q[0]]
        torso_point.time_from_start.sec = int(self.index * 0.1)
        torso_traj.points = [torso_point]

        arm_traj = JointTrajectory()
        arm_traj.joint_names = [f'arm_{i+1}_joint' for i in range(7)]
        arm_point = JointTrajectoryPoint()
        arm_point.positions = q[1:8].tolist()
        arm_point.time_from_start.sec = int(self.index * 0.1)
        arm_traj.points = [arm_point]

        self.torso_pub.publish(torso_traj)
        self.arm_pub.publish(arm_traj)
        self.get_logger().info(f'[PUB LOOP] idx {self.index % len(self.q_traj)} - Torso: {torso_point.positions}, Arm: {arm_point.positions}')

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = KinematicTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
