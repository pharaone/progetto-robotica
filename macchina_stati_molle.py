from enum import Enum
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading

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

class RobotStateMachineNode(Node):
    def __init__(self):
        super().__init__('robot_state_machine_node')
        self.get_logger().info("Avvio Robot State Machine...")
        self.stato_corrente = State.WAITING_FOR_ARUCO
        self.ultimo_stato = None

        self.publisher = self.create_publisher(Int32, '/command_topic', 10)
        self.create_subscription(Int32, '/completed_command_topic', self.completed_command_callback, 10)

        # Start the while loop in a separate thread to avoid blocking ROS
        self.thread = threading.Thread(target=self.state_machine_loop)
        self.thread.daemon = True
        self.thread.start()

    def state_machine_loop(self):
        while rclpy.ok():
            if self.stato_corrente != self.ultimo_stato:
                self.get_logger().info(f"Nuovo stato: {self.stato_corrente.name}")
                if self.stato_corrente == State.MOVE_TO_POSE_1:
                    self.move_to_pose_1()
                elif self.stato_corrente == State.MOVE_DOWN:
                    self.move_down()
                elif self.stato_corrente == State.GRIP_OBJECT:
                    self.grip_object()
                elif self.stato_corrente == State.RELEASE_OBJECT:
                    self.release_object()
                elif self.stato_corrente == State.MOVE_TO_HOME:
                    self.move_to_home()
                elif self.stato_corrente == State.INITIALIZE_GRIPPER_2:
                    self.initialize_gripper_2()
                elif self.stato_corrente == State.MOVE_TO_POSE_2:
                    self.move_to_pose_2()
                elif self.stato_corrente == State.GRIP_OBJECT_2:
                    self.grip_object_2()
                elif self.stato_corrente == State.RELEASE_OBJECT_2:
                    self.release_object_2()
                elif self.stato_corrente == State.MOVE_TO_HOME_2:
                    self.move_to_home_2()
                self.ultimo_stato = self.stato_corrente
            # Sleep to avoid maxing out the CPU (adjust as needed)
            time.sleep(0.1)

    def completed_command_callback(self, msg):
        # Logica per passare allo stato successivo in base al comando completato
        if msg.data == State.WAITING_FOR_ARUCO.value:
            self.stato_corrente = State.MOVE_TO_POSE_1
        elif msg.data == State.MOVE_TO_POSE_1.value:
            self.stato_corrente = State.MOVE_DOWN
        elif msg.data == State.MOVE_DOWN.value:
            self.stato_corrente = State.GRIP_OBJECT
        elif msg.data == State.GRIP_OBJECT.value:
            self.stato_corrente = State.RELEASE_OBJECT
        elif msg.data == State.RELEASE_OBJECT.value:
            self.stato_corrente = State.MOVE_TO_HOME
        elif msg.data == State.MOVE_TO_HOME.value:
            self.stato_corrente = State.INITIALIZE_GRIPPER_2
        elif msg.data == State.INITIALIZE_GRIPPER_2.value:
            self.stato_corrente = State.MOVE_TO_POSE_2
        elif msg.data == State.MOVE_TO_POSE_2.value:
            self.stato_corrente = State.GRIP_OBJECT_2
        elif msg.data == State.GRIP_OBJECT_2.value:
            self.stato_corrente = State.RELEASE_OBJECT_2
        elif msg.data == State.RELEASE_OBJECT_2.value:
            self.stato_corrente = State.MOVE_TO_HOME_2
        elif msg.data == State.MOVE_TO_HOME_2.value:
            self.stato_corrente = State.WAITING_FOR_ARUCO

    def move_to_pose_1(self):
        self.get_logger().info("Muovo verso la posa 1.")
        msg = Int32()
        msg.data = State.MOVE_TO_POSE_1.value
        self.publisher.publish(msg)
    def move_down(self):
        self.get_logger().info("Muovo verso il basso.")
        msg = Int32()
        msg.data = State.MOVE_DOWN.value
        self.publisher.publish(msg)
    def grip_object(self): pass
    def release_object(self): pass
    def move_to_home(self): pass
    def initialize_gripper_2(self): pass
    def move_to_pose_2(self): pass
    def grip_object_2(self): pass
    def release_object_2(self): pass
    def move_to_home_2(self): pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
