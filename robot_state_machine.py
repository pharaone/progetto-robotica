from enum import Enum
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

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
    INITIALIZE_GRIPPER = 16
    MOVE_UP_PRINGLES = 17

class RobotStateMachineNode(Node):
    def __init__(self):


        super().__init__('robot_state_machine_node')
        self.get_logger().info("Avvio Robot State Machine...")
        self.stato_corrente = State.WAITING_FOR_ARUCO
        self.ultimo_stato = State.WAITING_FOR_ARUCO
        self.current_target = 1

        self.publisher = self.create_publisher(Int32, '/command_topic', 10)

        self.create_subscription(Int32,'/completed_command_topic', self.completed_command_callback, 10)

        self.timer = self.create_timer(1.0, self.loop)

    def loop(self):         
        if self.stato_corrente != self.ultimo_stato:
            self.get_logger().info(f"Stato corrente: {self.stato_corrente}")
            if self.stato_corrente == State.MOVE_TO_POSE_1:
                self.move_to_pose_1()
            elif self.stato_corrente == State.MOVE_TO_POSE_2:
                self.move_to_pose_2()
            elif self.stato_corrente == State.GRIP_OBJECT:
                self.grip_object()
            elif self.stato_corrente == State.MOVE_UP_COLA:
                self.move_up_cola()
            elif self.stato_corrente == State.MOVE_COLA_TO_FINISH:
                self.move_cola_to_finish()
            elif self.stato_corrente == State.RELEASE_OBJECT:
                self.release_object()
            elif self.stato_corrente == State.MOVE_TO_HOME:
                self.move_to_home()
            elif self.stato_corrente == State.MOVE_TO_POSE_3:
                self.move_to_pose_3()
            elif self.stato_corrente == State.MOVE_TO_POSE_4:
                self.move_to_pose_4()
            elif self.stato_corrente == State.GRIP_OBJECT_2:
                self.grip_object_2()
            elif self.stato_corrente == State.MOVE_UP_PRINGLES:
                self.move_up_pringles()
            elif self.stato_corrente == State.MOVE_PRINGLES_TO_FINISH:
                self.move_pringles_to_finish()
            elif self.stato_corrente == State.RELEASE_OBJECT_2:
                self.release_object_2()
            elif self.stato_corrente == State.MOVE_TO_HOME_2:
                self.move_to_home_2()
            self.ultimo_stato = self.stato_corrente
        else:
            self.get_logger().info("Nessun cambiamento di stato.")

    def completed_command_callback(self, msg):
        self.get_logger().info("Ricevuto comando completato" + str(msg.data))
        if msg.data == State.WAITING_FOR_ARUCO.value:
            self.stato_corrente = State.MOVE_TO_POSE_1

        elif msg.data == State.MOVE_TO_POSE_1.value:
            self.stato_corrente = State.MOVE_TO_POSE_2
            
        elif msg.data == State.MOVE_TO_POSE_2.value:
            self.stato_corrente = State.GRIP_OBJECT
        elif msg.data == State.GRIP_OBJECT.value:
            self.stato_corrente = State.MOVE_UP_COLA

        elif msg.data == State.MOVE_UP_COLA.value:
            self.stato_corrente = State.MOVE_COLA_TO_FINISH

        elif msg.data == State.MOVE_COLA_TO_FINISH.value:
            self.stato_corrente = State.RELEASE_OBJECT

        elif msg.data == State.RELEASE_OBJECT.value:
            self.stato_corrente = State.MOVE_TO_HOME

        elif msg.data == State.MOVE_TO_HOME.value:
            self.stato_corrente = State.MOVE_TO_POSE_3

        elif msg.data == State.MOVE_TO_POSE_3.value:
            self.stato_corrente = State.MOVE_TO_POSE_4

        elif msg.data == State.MOVE_TO_POSE_4.value:
            self.stato_corrente = State.GRIP_OBJECT_2

        elif msg.data == State.GRIP_OBJECT_2.value:
            self.stato_corrente = State.MOVE_UP_PRINGLES
        
        elif msg.data == State.MOVE_UP_PRINGLES.value:
            self.stato_corrente = State.MOVE_PRINGLES_TO_FINISH

        elif msg.data == State.MOVE_PRINGLES_TO_FINISH.value:
            self.stato_corrente = State.RELEASE_OBJECT_2

        elif msg.data == State.RELEASE_OBJECT_2.value:
            self.stato_corrente = State.MOVE_TO_HOME_2

    def initialize_gripper(self):
        self.get_logger().info("Inizializzo gripper.")
        self.stato_corrente = State.MOVE_TO_POSE_1
        pass

    def initialize_gripper_2(self):
        self.get_logger().info("Inizializzo gripper 2.")
        self.stato_corrente = State.MOVE_TO_POSE_3
        pass

    def move_to_pose_1(self):
        self.get_logger().info("Muovo verso la posa 1.")
        msg = Int32()
        msg.data = State.MOVE_TO_POSE_1.value
        self.publisher.publish(msg)
        
    def move_to_pose_2(self):
        self.get_logger().info("Muovo verso la posa 2.")
        msg = Int32()
        msg.data = State.MOVE_TO_POSE_2.value
        self.publisher.publish(msg)

    def grip_object(self):
        self.get_logger().info("Chiudo il gripper per la coca.")
        msg = Int32()
        msg.data = State.GRIP_OBJECT.value
        self.publisher.publish(msg)

    def move_up_cola(self):
        self.get_logger().info("Muovo verso l'alto la coca.")
        msg = Int32()
        msg.data = State.MOVE_UP_COLA.value
        self.publisher.publish(msg)

    def move_cola_to_finish(self):
        self.get_logger().info("Muovo la coca verso la posizione finale.")
        msg = Int32()
        msg.data = State.MOVE_COLA_TO_FINISH.value
        self.publisher.publish(msg)

    def release_object(self):
        self.get_logger().info("Apro il gripper per la coca.")
        msg = Int32()
        msg.data = State.RELEASE_OBJECT.value
        self.publisher.publish(msg)

    def move_to_home(self):
        self.get_logger().info("Torno alla posizione iniziale.")
        msg = Int32()
        msg.data = State.MOVE_TO_HOME.value
        self.publisher.publish(msg)


        #### # Gestione delle patatine

    def grip_object_2(self):
        self.get_logger().info("Chiudo il gripper per le patatine.")
        msg = Int32()
        msg.data = State.GRIP_OBJECT_2.value
        self.publisher.publish(msg)

    def release_object_2(self):
        self.get_logger().info("Apro il gripper per le patatine.")
        msg = Int32()
        msg.data = State.RELEASE_OBJECT_2.value
        self.publisher.publish(msg)

    def move_to_pose_3(self):
        self.get_logger().info("Muovo verso la posa 3.")
        msg = Int32()
        msg.data = State.MOVE_TO_POSE_3.value
        self.publisher.publish(msg)

    def move_to_pose_4(self):
        self.get_logger().info("Muovo verso la posa 4.")
        msg = Int32()
        msg.data = State.MOVE_TO_POSE_4.value
        self.publisher.publish(msg)

    def move_up_pringles(self):
        self.get_logger().info("Muovo verso l'alto le patatine.")
        msg = Int32()
        msg.data = State.MOVE_UP_PRINGLES.value
        self.publisher.publish(msg)

    def move_pringles_to_finish(self):
        self.get_logger().info("Muovo le patatine verso la posizione finale.")
        msg = Int32()
        msg.data = State.MOVE_PRINGLES_TO_FINISH.value
        self.publisher.publish(msg)

    def move_to_home_2(self):
        self.get_logger().info("Torno alla posizione iniziale dopo le patatine.")
        msg = Int32()
        msg.data = State.MOVE_TO_HOME_2.value
        self.publisher.publish(msg)

                

    

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()