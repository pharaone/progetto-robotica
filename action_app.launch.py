from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description ( ) :
    ld = LaunchDescription ( )
    action_client=Node(package = "tiago_head" , executable="head_action_client")
    action_server=Node(package="tiago_head" , executable="head_action_server" )
    aruco_pose_estimator_node = Node(package="aruco_pose_estimator", executable="aruco_pose_estimator_node")
    to_base = Node(package="aruco_pose_estimator", executable ="to_base") 
    ld.add_action(action_client)
    ld.add_action(action_server)
    ld.add_action(aruco_pose_estimator_node)
    ld.add_action(to_base)
    return ld
