import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class RobotStateMachineNode(Node):
    def __init__(self):
        super().__init__('robot_state_machine_node')
        self.stato_corrente = "initialize_gripper"
        self.current_target = 1

        self.subscription_event = self.create_subscription(
            String, '/event_topic', self.event_callback, 10)
        self.subscription_pose_1 = self.create_subscription(
            PoseStamped, '/aruco_pose_1', self.aruco_pose_1_callback, 10)
        self.subscription_pose_2 = self.create_subscription(
            PoseStamped, '/aruco_pose_2', self.aruco_pose_2_callback, 10)
        self.subscription_arm = self.create_subscription(
            String, '/arm_controller/joint_trajectory', self.event_callback, 10)

        self.publisher = self.create_publisher(String, '/command_topic', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.aruco_pose_1 = None
        self.aruco_pose_2 = None

        self.pose_sequence = []
        self.current_pose_idx = 0

    def event_callback(self, msg):
        evento = msg.data
        self.get_logger().info(f"Evento ricevuto: {evento}")
        self.transition(evento)

    def aruco_pose_1_callback(self, msg):
        self.aruco_pose_1 = msg

    def aruco_pose_2_callback(self, msg):
        self.aruco_pose_2 = msg

    def transition(self, evento):
        metodo_stato = getattr(self, f"{self.stato_corrente}_state")
        nuovo_stato = metodo_stato(evento)
        if nuovo_stato != self.stato_corrente:
            self.get_logger().info(f"Transizione: {self.stato_corrente} -> {nuovo_stato}")
            self.stato_corrente = nuovo_stato
            self.publisher.publish(String(data=nuovo_stato))
        else:
            self.get_logger().info(f"Nessuna transizione per evento '{evento}' nello stato {self.stato_corrente}")

    def initialize_gripper_state(self, evento):
        if evento == "gripper_initialized":
            return "wait_for_command"
        self.get_logger().info("Sto aprendo il gripper all'avvio...")
        return self.stato_corrente

    def wait_for_command_state(self, evento):
        if evento == "command_received":
            return "detect_aruco"
        return self.stato_corrente

    def detect_aruco_state(self, evento):
        if evento == "aruco_detected":
            if self.current_target == 1 and self.aruco_pose_1 is None:
                self.get_logger().warn("ArUco 1 non ancora ricevuto!")
                return self.stato_corrente
            if self.current_target == 2 and self.aruco_pose_2 is None:
                self.get_logger().warn("ArUco 2 non ancora ricevuto!")
                return self.stato_corrente
            return "calculate_pose_with_offset"
        return self.stato_corrente

    def calculate_pose_with_offset_state(self, evento):
        if evento == "pose_calculated":
            if self.current_target == 1:
                base_pose = self.aruco_pose_1.pose
                header = self.aruco_pose_1.header
            else:
                base_posarrived_to_graspe = self.aruco_pose_2.pose
                header = self.aruco_pose_2.header

            # Costruisci la sequenza: sopra > indietro e abbasso > avanti
            pose_above = PoseStamped()
            pose_above.header = header
            pose_above.header.stamp = self.get_clock().now().to_msg()
            pose_above.pose.position.x = base_pose.position.x - 0.1
            pose_above.pose.position.y = base_pose.position.y
            pose_above.pose.position.z = base_pose.position.z
            pose_above.pose.orientation = base_pose.orientation

            pose_back = PoseStamped()
            pose_back.header = header
            pose_back.header.stamp = self.get_clock().now().to_msg()
            pose_back.pose.position.x = base_pose.position.x - 0.1
            pose_back.pose.position.y = base_pose.position.y
            pose_back.pose.position.z = base_pose.position.z - 0.1
            pose_back.pose.orientation = base_pose.orientation

            pose_forward = PoseStamped()
            pose_forward.header = header
            pose_forward.header.stamp = self.get_clock().now().to_msg()
            pose_forward.pose.position.x = base_pose.position.x
            pose_forward.pose.position.y = base_pose.position.y
            pose_forward.pose.position.z = base_pose.position.z - 0.1
            pose_forward.pose.orientation = base_pose.orientation

            self.pose_sequence = [pose_above, pose_back, pose_forward]
            self.current_pose_idx = 0

            # Pubblica SOLO la prima pose
            self.pose_pub.publish(self.pose_sequence[0])
            self.get_logger().info("Pubblicata prima pose della sequenza con offset.")
            return "move_to_grasp"
        return self.stato_corrente

    def move_to_grasp_state(self, evento):
        # Dopo ogni arrivo, pubblica la successiva
        if evento == "arrived_to_grasp":
            self.current_pose_idx += 1
            if self.current_pose_idx < len(self.pose_sequence):
                self.pose_pub.publish(self.pose_sequence[self.current_pose_idx])
                self.get_logger().info(f"Pubblicata pose {self.current_pose_idx+1} della sequenza.")
                return "move_to_grasp"
            else:
                return "go_ahead"
        return self.stato_corrente

    def go_ahead_state(self, evento):
        if evento == "in_position":
            return "close_gripper"
        return self.stato_corrente

    def close_gripper_state(self, evento):
        if evento == "gripper_closed":
            return "move_to_lift"
        return self.stato_corrente

    def move_to_lift_state(self, evento):
        if evento == "lifted":
            return "go_to_drop_zone"
        return self.stato_corrente

    def go_to_drop_zone_state(self, evento):
        if evento == "arrived_to_drop_zone":
            return "open_gripper"
        return self.stato_corrente

    def open_gripper_state(self, evento):
        if evento == "gripper_opened":
            return "task_done"
        return self.stato_corrente

    def task_done_state(self, evento):
        self.get_logger().info(f"Task completato per oggetto {self.current_target}.")
        if self.current_target == 1:
            self.get_logger().info("Passo al secondo oggetto.")
            self.current_target = 2
            return "wait_for_command"
        else:
            self.get_logger().info("Tutti i task completati.")
            return self.stato_corrente

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()