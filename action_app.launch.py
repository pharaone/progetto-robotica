from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    # Nodi ROS 2 da lanciare
    action_client = Node(
        package="tiago_head",
        executable="head_action_client"
    )
    action_server = Node(
        package="tiago_head",
        executable="head_action_server"
    )
    aruco_pose_estimator_node = Node(
        package="aruco_pose_estimator",
        executable="aruco_pose_estimator_node"
    )
    to_base = Node(
        package="aruco_pose_estimator",
        executable="to_base"
    )

    kinematic1 = Node (
        package="kinematic",
        executable="kinematic1",
    )

    robot_machine_state = Node (
        package="kinematic",
        executable="robot_state_machine",
    )





    # Traiettoria 1: posizione intermedia del braccio (dopo 8 secondi)
    send_arm_trajectory_1 = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ['
                    '"arm_1_joint", "arm_2_joint", "arm_3_joint", '
                    '"arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], '
                    '"points": ['
                    '{"positions": [0.0003836728901962516, -0.0001633239063343339, -9.037018213753356e-06, '
                    '-6.145563957549172e-05, 4.409014973383307e-05, 0.0019643255648595925, 0.0004167305736686444], '
                    '"time_from_start": {"sec": 3}}'
                    ']}}'
                ],
                output='screen'
            )
        ]
    )

    # Torso dopo 8 secondi
    send_torso_trajectory = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/torso_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["torso_lift_joint"], '
                    '"points": [{"positions": [0.35], "time_from_start": {"sec": 3}}]}}'
                ],
                output='screen'
            )
        ]
    )



    # Traiettoria 2: posizione finale del braccio (dopo 11 secondi)
    send_arm_trajectory_2 = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ['
                    '"arm_1_joint", "arm_2_joint", "arm_3_joint", '
                    '"arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], '
                    '"points": ['
                    '{"positions": [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05], '
                    '"time_from_start": {"sec": 3}}'
                    ']}}'
                ],
                output='screen'
            )
        ]
    )

    # Aggiunta delle azioni alla LaunchDescription
    ld.add_action(action_client)
    ld.add_action(action_server)
    ld.add_action(aruco_pose_estimator_node)
    ld.add_action(to_base)
    ld.add_action(send_arm_trajectory_1)
    ld.add_action(send_torso_trajectory)
    ld.add_action(send_arm_trajectory_2)
    ld.add_action(kinematic1)
    ld.add_action(robot_machine_state)

    return ld

