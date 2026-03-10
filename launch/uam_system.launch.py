"""
uam_system.launch.py
--------------------
Launch file khởi động toàn bộ hệ thống UAM:
  1. Micro XRCE-DDS Agent (cầu nối PX4 ↔ ROS2)
  2. Node C++: Adaptive Backstepping + RBFNN (100 Hz)
  3. Node Python: LSTM Predictive Feedforward (20 Hz)
  4. Node Python: Arm Dynamics Newton-Euler (50 Hz)

Sử dụng:
  ros2 launch uam_controller uam_system.launch.py
  ros2 launch uam_controller uam_system.launch.py sim:=true
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             ExecuteProcess, GroupAction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   PythonExpression)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare('uam_controller')

    # ── Tham số launch ──
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true = môi trường mô phỏng Gazebo SITL'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([pkg_share, 'models', 'lstm_uam_weights.pth']),
        description='Đường dẫn tới file trọng số LSTM'
    )
    hover_height_arg = DeclareLaunchArgument(
        'hover_height',
        default_value='1.5',
        description='Độ cao lơ lửng mặc định [m]'
    )

    sim         = LaunchConfiguration('sim')
    model_path  = LaunchConfiguration('model_path')
    hover_height= LaunchConfiguration('hover_height')

    # ── Cấu hình tham số bộ điều khiển ──
    controller_params = {
        'mass_total':    2.5,
        'Ixx':           0.029,
        'Iyy':           0.029,
        'Izz':           0.055,
        'K1_pos':        3.0,
        'K2_vel':        2.5,
        'K1_att':        4.0,
        'K2_rate':       3.0,
        'rbfnn_neurons': 25,
        'rbfnn_lr':      0.08,
        'rbfnn_eta':     0.005,
        'rbfnn_width':   1.5,
    }

    # ── Node 1: Micro XRCE-DDS Agent ──
    # Chạy agent để PX4 Firmware có thể giao tiếp với ROS2
    xrce_dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_dds_agent',
        output='screen',
        condition=UnlessCondition(sim)
    )

    # Chạy agent qua UDP cho môi trường mô phỏng SITL
    xrce_dds_agent_sim = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_dds_agent_sim',
        output='screen',
        condition=IfCondition(sim)
    )

    # ── Node 2: Adaptive Backstepping + RBFNN (C++) ──
    backstepping_node = Node(
        package='uam_controller',
        executable='uam_backstepping_rbfnn_node',
        name='uam_adaptive_controller',
        output='screen',
        parameters=[
            controller_params,
            {'hover_height': hover_height}
        ],
        remappings=[
            ('/fmu/in/offboard_control_mode',   '/fmu/in/offboard_control_mode'),
            ('/fmu/in/vehicle_torque_setpoint',  '/fmu/in/vehicle_torque_setpoint'),
            ('/fmu/in/vehicle_thrust_setpoint',  '/fmu/in/vehicle_thrust_setpoint'),
            ('/fmu/out/vehicle_odometry',        '/fmu/out/vehicle_odometry'),
        ],
        # Gắn core CPU 0 + 1 cho node điều khiển quan trọng
        additional_env={'ROS_DOMAIN_ID': '0'}
    )

    # ── Node 3: LSTM Predictive Feedforward (Python) ──
    lstm_node = Node(
        package='uam_controller',
        executable='lstm_feedforward_node.py',
        name='lstm_predictive_node',
        output='screen',
        parameters=[{
            'model_path':    model_path,
            'seq_len':       10,
            'scale_factor':  1.0,
            'enable_filter': True
        }]
    )

    # ── Node 4: Arm Dynamics Newton-Euler (Python) ──
    arm_dynamics_node = Node(
        package='uam_controller',
        executable='arm_dynamics_node.py',
        name='arm_dynamics_node',
        output='screen'
    )

    # ── Trễ khởi động để đảm bảo DDS bridge sẵn sàng ──
    delayed_backstepping = TimerAction(
        period=2.0,
        actions=[backstepping_node]
    )
    delayed_lstm = TimerAction(
        period=3.0,
        actions=[lstm_node]
    )
    delayed_arm_dynamics = TimerAction(
        period=2.5,
        actions=[arm_dynamics_node]
    )

    return LaunchDescription([
        sim_arg,
        model_path_arg,
        hover_height_arg,
        xrce_dds_agent,
        xrce_dds_agent_sim,
        delayed_backstepping,
        delayed_lstm,
        delayed_arm_dynamics,
    ])
