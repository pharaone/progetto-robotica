import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class TestRobotNode(Node):
    def __init__(self):
        super().__init__('test_robot_node')

        self.event_pub = self.create_publisher(String, '/event_topic', 10)
        self.pose1_pub = self.create_publisher(PoseStamped, '/aruco_pose_1', 10)
        self.pose2_pub = self.create_publisher(PoseStamped, '/aruco_pose_2', 10)

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.step = 0

    def timer_callback(self):
        # Step-by-step, simula tutto il flusso che vuoi vedere nella tua macchina a stati!
        if self.step == 0:
            self.log_and_pub('gripper_initialized')
        elif self.step == 1:
            self.log_and_pub('command_received')
        elif self.step == 2:
            self.get_logger().info('Publishing aruco_detected event and ArUco poses')

            pose1 = PoseStamped()
            pose1.header.stamp = self.get_clock().now().to_msg()
            pose1.header.frame_id = 'map'
            pose1.pose.position.x = 1.0
            pose1.pose.position.y = 2.0
            pose1.pose.position.z = 0.5
            pose1.pose.orientation.x = 0.0
            pose1.pose.orientation.y = 0.0
            pose1.pose.orientation.z = 0.0
            pose1.pose.orientation.w = 1.0
            self.pose1_pub.publish(pose1)

            pose2 = PoseStamped()
            pose2.header.stamp = self.get_clock().now().to_msg()
            pose2.header.frame_id = 'map'
            pose2.pose.position.x = 3.0
            pose2.pose.position.y = 4.0
            pose2.pose.position.z = 0.5
            pose2.pose.orientation.x = 0.0
            pose2.pose.orientation.y = 0.0
            pose2.pose.orientation.z = 0.0
            pose2.pose.orientation.w = 1.0
            self.pose2_pub.publish(pose2)

            self.log_and_pub('aruco_detected')
        elif self.step == 3:
            self.log_and_pub('pose_calculated')
        elif self.step == 4:
            self.log_and_pub('arrived_to_grasp')
        elif self.step == 5:
            self.log_and_pub('arrived_to_grasp')  # per sequenza di pose
        elif self.step == 6:
            self.log_and_pub('arrived_to_grasp')  # per sequenza di pose
        elif self.step == 7:
            self.log_and_pub('in_position')
        elif self.step == 8:
            self.log_and_pub('gripper_closed')
        elif self.step == 9:
            self.log_and_pub('lifted')
        elif self.step == 10:
            self.log_and_pub('arrived_to_drop_zone')
        elif self.step == 11:
            self.log_and_pub('gripper_opened')
        elif self.step == 12:
            self.get_logger().info('Test sequence completed, shutting down...')
            rclpy.shutdown()
        self.step += 1

    def log_and_pub(self, event):
        self.get_logger().info(f'Publishing event: {event}')
        self.event_pub.publish(String(data=event))

def main(args=None):
    rclpy.init(args=args)
    node = TestRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
