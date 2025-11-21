#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = 'hyper_teleop'
    pkg_share = get_package_share_directory(package_name)
    
    # ✅ Configurar rutas para Ignition Gazebo (NO es GAZEBO_MODEL_PATH)
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    
    # Para Ignition Gazebo Fortress usa IGN_GAZEBO_RESOURCE_PATH
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = pkg_share_path
    
    # También agrega la ruta directa a los meshes
    meshes_path = os.path.join(pkg_share, 'meshes')
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] += os.pathsep + meshes_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = meshes_path
    
    # Paths
    world_path = os.path.join(pkg_share, 'worlds', 'piscina_world.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'salvavidas.urdf')
    teleop_config = os.path.join(pkg_share, 'config', 'teleop_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'teleop_rviz.rviz')
    sdf_temp_path = '/tmp/salvavidas.sdf'
    
    # 1️⃣ Launch Gazebo Fortress
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen',
        shell=False,
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}
    )
    
    # 2️⃣ Convert URDF → SDF (ejecutar PRIMERO)
    convert_urdf_to_sdf = ExecuteProcess(
        cmd=['bash', '-c', f'ign sdf -p {urdf_path} > {sdf_temp_path}'],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}
    )
    
    # 3️⃣ Spawn robot usando ros_gz_sim (método alternativo más confiable)
    spawn_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=convert_urdf_to_sdf,
            on_exit=[
                TimerAction(
                    period=8.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                'ign', 'service',
                                '-s', '/world/piscina_world/create',
                                '--reqtype', 'ignition.msgs.EntityFactory',
                                '--reptype', 'ignition.msgs.Boolean',
                                '--timeout', '1000',
                                '--req', ( 
                                        f'sdf_filename: "{sdf_temp_path}"'
                                        'pose: { position: { x: 0, y: 0, z: 0.05 } }'
                                )
                            ],
                            output='screen'
                        )
                    ]
                )
            ]
        )
    )
    
    # 4️⃣ Bridge for cmd_vel (ROS2 -> Gazebo)
    bridge_cmd_vel = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
                ],
                output='screen'
            )
        ]
    )
    
    # 5️⃣ Bridge for odometry (Gazebo -> ROS2)
    bridge_odom = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
                ],
                output='screen'
            )
        ]
    )
    
    # Bridge para joint_states (Gazebo -> ROS 2)
    bridge_joint_states = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'
                ],
                output='screen'
            )
        ]
    )
    
    # Bridge para TF (Gazebo -> ROS 2)
    bridge_tf = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # Pasa los TF de Gazebo a ROS 2
                    '/model/salvavidas/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
                ],
                remappings=[
                    # Reenvía las transformaciones a /tf, que es el estándar de ROS 2
                    ('/model/salvavidas/tf', '/tf')
                ],
                output='screen'
            )
        ]
    )

    # 6️⃣ Start joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }],
        output='screen'
    )
    
    # 7️⃣ Start teleop node
    teleop_node = Node(
        package='hyper_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[teleop_config],
        output='screen'
    )

    # 8️⃣ Robot State Publisher (para TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # 9️⃣ RViz visualization
    rviz_node = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': True}],
                output='screen',
                additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
            )
        ]
    )

    # Bridge node
    bridge_node = Node(
        package='hyper_teleop',
        executable='bridge_sus',
        name='bridge_node',
        output='screen'
    )

    
    return LaunchDescription([
        ign_gazebo,
        convert_urdf_to_sdf,
        spawn_robot,
        bridge_cmd_vel,
        bridge_odom,
        bridge_joint_states,
        bridge_tf,
        joy_node,
        teleop_node,
        robot_state_publisher,
        rviz_node,
        bridge_node
    ])
