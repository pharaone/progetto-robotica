#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np


class ArucoPoseTFTransformer(Node):
    def __init__(self):
        super().__init__('aruco_pose_transformer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Lista degli ID marker previsti
        self.marker_ids = [1, 2, 3, 4]

        # Salva le ultime pose ricevute per ogni marker
        self.last_poses = {}

        # Crea i publisher e i subscriber per ogni marker
        self.pose_publishers = {}
        for marker_id in self.marker_ids:
            topic_in = f'/aruco_marker_pose_{marker_id}'
            topic_out = f'/aruco_pose_{marker_id}'
            self.create_subscription(PoseStamped, topic_in, self.make_callback(marker_id), 10)
            self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, topic_out, 10)

        # Timer per trasformare e pubblicare ogni secondo
        self.create_timer(1.0, self.transform_all_poses)

    def make_callback(self, marker_id):
        def callback(msg: PoseStamped, marker_id=marker_id):  # Cattura sicura
            self.last_poses[marker_id] = msg
            self.get_logger().info(
                f"[Marker {marker_id}] Ricevuta posa: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
            )
        return callback

    def transform_all_poses(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='base_footprint',
                source_frame='head_front_camera_color_optical_frame',
                time=rclpy.time.Time()
            )

            trans = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            quat_tf = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            rot_tf = R.from_quat(quat_tf).as_matrix()

            for marker_id, pose in self.last_poses.items():
                pos_marker_cam = np.array([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ])
                quat_marker_cam = np.array([
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                ])
                rot_marker_cam = R.from_quat(quat_marker_cam).as_matrix()

                # Trasformazione posizione e orientamento
                pos_marker_base = rot_tf @ pos_marker_cam + trans
                rot_marker_base = R.from_matrix(rot_tf @ rot_marker_cam)
                quat_marker_base = rot_marker_base.as_quat()

                # Costruisci e pubblica messaggio trasformato
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'base_footprint'
                pose_msg.pose.position.x = pos_marker_base[0]
                pose_msg.pose.position.y = pos_marker_base[1]
                pose_msg.pose.position.z = pos_marker_base[2]
                pose_msg.pose.orientation.x = quat_marker_base[0]
                pose_msg.pose.orientation.y = quat_marker_base[1]
                pose_msg.pose.orientation.z = quat_marker_base[2]
                pose_msg.pose.orientation.w = quat_marker_base[3]

                self.pose_publishers[marker_id].publish(pose_msg)
                self.get_logger().info(f"[TF] Marker {marker_id}: pubblicata posa trasformata.")

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"[TF] Trasformazione non trovata: {str(e)}")


def main():
    rclpy.init()
    node = ArucoPoseTFTransformer()
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