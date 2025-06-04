from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    ld = LaunchDescription()

    # Nodi da lanciare subito
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

    # Avvio ritardato per kinematic1 e robot_state_machine
    kinematic1_timer = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="kinematic",
                executable="kinematic1"
            )
        ]
    )

    robot_machine_state_timer = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="kinematic",
                executable="robot_state_machine"
            )
        ]
    )
    
    # Traiettoria 1: posizione intermedia del braccio 
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
                    '{"positions": [0.00038, -0.00016, -0.000009, '
                    '-0.000061, 0.000044, 0.00196, 0.00042], '
                    '"time_from_start": {"sec": 3}}'
                    ']}}'
                ],
                output='screen'
            )
        ]
    )

    # Movimento del torso
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

    # Traiettoria 2: posizione finale del braccio 
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

    #Apertura gripper dopo 17s
    gripper_open = TimerAction(
        period=17.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/gripper_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    '{"trajectory": {"joint_names": ["gripper_left_finger_joint", "gripper_right_finger_joint"], '
                    '"points": [{"positions": [0.044, 0.044], "time_from_start": {"sec": 2}}]}}'
                ],
                output='screen'
            )
        ]

    )


    # Aggiunta di tutte le azioni e nodi alla launch description
    ld.add_action(action_client)
    ld.add_action(action_server)
    ld.add_action(aruco_pose_estimator_node)
    ld.add_action(to_base)
    ld.add_action(kinematic1_timer)
    ld.add_action(robot_machine_state_timer)
    ld.add_action(send_arm_trajectory_1)
    ld.add_action(send_torso_trajectory)
    ld.add_action(send_arm_trajectory_2)
    ld.add_action(gripper_open)

    return ld
