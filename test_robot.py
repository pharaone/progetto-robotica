import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time

class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher_node')

        self.event_pub = self.create_publisher(String, '/event_topic', 10)
        self.pose1_pub = self.create_publisher(PoseStamped, '/aruco_pose_1', 10)
        self.pose2_pub = self.create_publisher(PoseStamped, '/aruco_pose_2', 10)

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.step = 0

    def timer_callback(self):
        if self.step == 0:
            self.get_logger().info('Publishing gripper_initialized event')
            self.event_pub.publish(String(data='gripper_initialized'))

        elif self.step == 1:
            self.get_logger().info('Publishing command_received event')
            self.event_pub.publish(String(data='command_received'))

        elif self.step == 2:
            self.get_logger().info('Publishing aruco_detected event and ArUco poses')

            # Publish ArUco pose 1
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

            # Publish ArUco pose 2
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

            # Publish event
            self.event_pub.publish(String(data='aruco_detected'))

        elif self.step == 3:
            self.get_logger().info('Publishing pose_calculated event')
            self.event_pub.publish(String(data='pose_calculated'))

        elif self.step == 4:
            self.get_logger().info('Publishing arrived_to_grasp event')
            self.event_pub.publish(String(data='arrived_to_grasp'))

        elif self.step == 5:
            self.get_logger().info('Publishing in_position event')
            self.event_pub.publish(String(data='in_position'))

        elif self.step == 6:
            self.get_logger().info('Publishing gripper_closed event')
            self.event_pub.publish(String(data='gripper_closed'))

        elif self.step == 7:
            self.get_logger().info('Publishing lifted event')
            self.event_pub.publish(String(data='lifted'))

        elif self.step == 8:
            self.get_logger().info('Publishing arrived_to_drop_zone event')
            self.event_pub.publish(String(data='arrived_to_drop_zone'))

        elif self.step == 9:
            self.get_logger().info('Publishing gripper_opened event')
            self.event_pub.publish(String(data='gripper_opened'))

        elif self.step == 10:
            self.get_logger().info('Test sequence completed, shutting down...')
            rclpy.shutdown()

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
