from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    # Nodi ROS 2 da lanciare (senza output)
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

    # Comando per inviare la traiettoria al torso dopo 9 secondi
    send_torso_trajectory = TimerAction(
        period=8.0,  # Attendi 8 secondi prima di eseguire l'azione del torso
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/torso_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["torso_lift_joint"], "points": [{"positions": [0.35], "time_from_start": {"sec": 3}}]}}'
                ],
                output='screen'
            )
        ]
    )

    # Comando per inviare la traiettoria al braccio dopo 11 secondi
    send_arm_trajectory = TimerAction(
        period=8.0,  # DA VERIFICARE
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"], "points": [{"positions": [0.07, 0.1, -3.1, 1.36, 2.05, 0.01, -0.05], "time_from_start": {"sec": 3}}]}}'
                ],
                output='screen'
            )
        ]
    )

    # Aggiunta di tutte le azioni alla LaunchDescription
    ld.add_action(action_client)
    ld.add_action(action_server)
    ld.add_action(aruco_pose_estimator_node)
    ld.add_action(to_base)
    ld.add_action(send_torso_trajectory)
    ld.add_action(send_arm_trajectory)

    return ld

