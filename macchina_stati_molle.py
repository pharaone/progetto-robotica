import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class RobotStateMachineNode(Node):
    def __init__(self):
        super().__init__('robot_state_machine_node')

        self.stato_corrente = "initialize_gripper"
        self.current_target = 1

        # Subscribers (se servono altri event/aruco topic aggiungili)
        self.subscription_event = self.create_subscription(
            String, '/event_topic', self.event_callback, 10)
        self.subscription_pose_1 = self.create_subscription(
            PoseStamped, '/aruco_pose_1', self.aruco_pose_1_callback, 10)
        self.subscription_pose_2 = self.create_subscription(
            PoseStamped, '/aruco_pose_2', self.aruco_pose_2_callback, 10)

        # PUBs per TaskPlanner
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.command_pub = self.create_publisher(String, '/command_topic', 10)

        self.aruco_pose_1 = None
        self.aruco_pose_2 = None

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
            # Puoi anche pubblicare sul /event_topic qui se vuoi
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
            # Esempio di preparazione della pose
            if self.current_target == 1:
                base_pose = self.aruco_pose_1.pose
                header = self.aruco_pose_1.header
            else:
                base_pose = self.aruco_pose_2.pose
                header = self.aruco_pose_2.header

            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = base_pose.position.x - 0.1
            pose_stamped.pose.position.y = base_pose.position.y
            pose_stamped.pose.position.z = base_pose.position.z
            pose_stamped.pose.orientation = base_pose.orientation

            self.get_logger().info(f"Pubblico pose offset su /target_pose e comando su /command_topic")
            self.pose_pub.publish(pose_stamped)
            self.command_pub.publish(String(data="grasp"))

            return "move_to_grasp"
        return self.stato_corrente

    def move_to_grasp_state(self, evento):
        if evento == "arrived_to_grasp":
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
