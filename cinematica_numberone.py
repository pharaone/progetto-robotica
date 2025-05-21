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

        # Carica modello dal tuo URDF (metti il percorso giusto)
        urdf_loc = '/home/davide/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)
        self.get_logger().info('[INIT] Robot URDF caricato')

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # Publishers
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.torso_pub = self.create_publisher(JointTrajectory, '/torso_controller/joint_trajectory', 10)

        # Stato
        self.q0 = None   # posizione iniziale (salvata solo una volta)
        self.target_pose = None
        self.traj_ready = False
        self.q_traj = None
        self.index = 0

    def joint_states_callback(self, msg):
        if self.q0 is not None:
            return  # Già acquisito!

        joint_names = ['torso_lift_joint'] + [f'arm_{i+1}_joint' for i in range(7)]
        q_init = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                q_init.append(msg.position[idx])
            else:
                q_init.append(0.0)
        self.q0 = np.array(q_init)
        self.get_logger().info(f"[JOINT INIT] Configurazione iniziale acquisita: {q_init}")
        self.try_plan_traj()

    def target_pose_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(f"[POSE] Target pose ricevuta: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")
        self.try_plan_traj()

    def try_plan_traj(self):
        if self.q0 is not None and self.target_pose is not None and not self.traj_ready:
            # Costruisci SE3 target
            pose = self.target_pose.pose
            T0 = self.robot.fkine(self.q0)
            Tf = SE3(pose.position.x, pose.position.y, pose.position.z)
            self.get_logger().info(f"[TRAJ] Calcolo traiettoria tra:\n  T0={T0}\n  Tf={Tf}")
            N = 60
            Ts = ctraj(T0, Tf, N)

            q_traj = []
            q_curr = self.q0
            for T in Ts:
                sol = self.robot.ik_NR(T, q0=q_curr, pinv=True)
                if sol is not None and len(sol) > 0:
                    q_curr = sol[0]
                q_traj.append(q_curr)
            self.q_traj = np.array(q_traj)
            self.get_logger().info(f"[TRAJ] Traiettoria calcolata con {len(q_traj)} punti.")
            self.traj_ready = True
            self.timer = self.create_timer(0.1, self.timer_callback)  # Start sending traj

    def timer_callback(self):
        if self.q_traj is None or self.index >= len(self.q_traj):
            self.get_logger().info('[TRAJ] Traiettoria completata.')
            self.destroy_timer(self.timer)
            return

        q = self.q_traj[self.index]
        # Torso su un topic, Arm su un altro (assume torso=idx0, arm=idx1:8)
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
        self.get_logger().info(f'[PUB] step {self.index} - Torso: {torso_point.positions}, Arm: {arm_point.positions}')
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
