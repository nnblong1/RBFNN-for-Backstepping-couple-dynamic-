"""
uam_system.launch.py
--------------------
Khởi động toàn bộ hệ thống UAM.

NGUYÊN TẮC THIẾT KẾ:
  - File này KHÔNG định nghĩa bất kỳ giá trị tham số nào.
  - Mọi tham số điều khiển nằm trong:
      config/uam_controller_params.yaml   (mặc định)
      config/uam_controller_params_sim.yaml   (khi sim:=true)
  - File này chỉ quyết định: node nào chạy, chạy với config nào,
    kết nối topic ra sao, và thứ tự khởi động.

Cách dùng:
  # Chạy phần cứng thật
  ros2 launch uam_controller uam_system.launch.py

  # Chạy Gazebo SITL
  ros2 launch uam_controller uam_system.launch.py sim:=true

  # Chỉ định file config khác
  ros2 launch uam_controller uam_system.launch.py \\
      config_file:=/path/to/my_custom_params.yaml

  # Chỉ định file trọng số LSTM khác
  ros2 launch uam_controller uam_system.launch.py \\
      model_path:=/path/to/retrained_weights.pth
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('uam_controller')

    # ═══════════════════════════════════════════════════════════
    #  KHAI BÁO ARGUMENT
    #  Chỉ có 3 loại argument hợp lệ ở đây:
    #    1. Chọn môi trường (sim / hardware)
    #    2. Chọn file config
    #    3. Chọn file model (phụ thuộc đường dẫn máy)
    # ═══════════════════════════════════════════════════════════

    arg_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true = Gazebo SITL | false = phần cứng thật'
    )

    arg_config = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
            [pkg_share, 'config', 'uam_controller_params.yaml']
        ),
        description='Đường dẫn file YAML chứa toàn bộ tham số hệ thống'
    )

    arg_model = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution(
            [pkg_share, 'models', 'lstm_uam_weights.pth']
        ),
        description='Đường dẫn file trọng số LSTM (.pth)'
    )

    sim         = LaunchConfiguration('sim')
    config_file = LaunchConfiguration('config_file')
    model_path  = LaunchConfiguration('model_path')

    # ═══════════════════════════════════════════════════════════
    #  NODE 0 – Micro XRCE-DDS Agent
    #  Cầu nối PX4 Firmware ↔ ROS2 qua Micro XRCE-DDS
    #  Hardware: UART /dev/ttyAMA0 @ 921600 bps
    #  Sim    : UDP port 8888
    # ═══════════════════════════════════════════════════════════

    xrce_hardware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        name='micro_xrce_dds_agent',
        output='screen',
        condition=UnlessCondition(sim)
    )

    xrce_sim = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_dds_agent_sim',
        output='screen',
        condition=IfCondition(sim)
    )

    # ═══════════════════════════════════════════════════════════
    #  NODE 1 – Adaptive Backstepping + RBFNN  (C++, 100 Hz)
    #  Tham số: ĐỌC HOÀN TOÀN từ config_file
    #  Không có parameters=[{...}] inline ở đây
    # ═══════════════════════════════════════════════════════════

    backstepping_node = Node(
        package='uam_controller',
        executable='uam_backstepping_rbfnn_node',
        name='uam_adaptive_controller',
        output='screen',
        parameters=[config_file],   # ← duy nhất, không thêm gì nữa
        remappings=[
            ('/fmu/in/offboard_control_mode',   '/fmu/in/offboard_control_mode'),
            ('/fmu/in/vehicle_torque_setpoint',  '/fmu/in/vehicle_torque_setpoint'),
            ('/fmu/in/vehicle_thrust_setpoint',  '/fmu/in/vehicle_thrust_setpoint'),
            ('/fmu/out/vehicle_odometry',        '/fmu/out/vehicle_odometry'),
            ('/fmu/out/vehicle_status',          '/fmu/out/vehicle_status'),
        ],
        additional_env={'ROS_DOMAIN_ID': '0'}
    )

    # ═══════════════════════════════════════════════════════════
    #  NODE 2 – LSTM Predictive Feedforward  (Python, 20 Hz)
    #  model_path là ngoại lệ duy nhất: phụ thuộc đường dẫn máy
    #  nên truyền qua argument thay vì hardcode trong YAML
    # ═══════════════════════════════════════════════════════════

    lstm_node = Node(
        package='uam_controller',
        executable='lstm_feedforward_node.py',
        name='lstm_predictive_node',
        output='screen',
        parameters=[
            config_file,                    # seq_len, scale_factor, enable_filter
            {'model_path': model_path},     # ngoại lệ: đường dẫn phụ thuộc máy
        ]
    )

    # ═══════════════════════════════════════════════════════════
    #  NODE 3 – Arm Dynamics Newton-Euler  (Python, 50 Hz)
    #  Tham số: ĐỌC HOÀN TOÀN từ config_file
    # ═══════════════════════════════════════════════════════════

    arm_dynamics_node = Node(
        package='uam_controller',
        executable='arm_dynamics_node.py',
        name='arm_dynamics_node',
        output='screen',
        parameters=[config_file]    # link_masses, link_lengths, dh_*
    )

    # ═══════════════════════════════════════════════════════════
    #  THỨ TỰ KHỞI ĐỘNG
    #  DDS Agent cần sẵn sàng trước khi các node ROS2 kết nối PX4
    #
    #  t=0s   DDS Agent khởi động
    #  t=2s   Backstepping node bắt đầu (chờ DDS ổn định)
    #  t=2.5s Arm Dynamics node bắt đầu
    #  t=3s   LSTM node bắt đầu (cần Backstepping đã chạy trước)
    # ═══════════════════════════════════════════════════════════

    delayed_backstepping = TimerAction(period=2.0,   actions=[backstepping_node])
    delayed_arm_dynamics = TimerAction(period=2.5,   actions=[arm_dynamics_node])
    delayed_lstm         = TimerAction(period=3.0,   actions=[lstm_node])

    return LaunchDescription([
        # Arguments
        arg_sim,
        arg_config,
        arg_model,
        # DDS bridge (khởi động ngay)
        xrce_hardware,
        xrce_sim,
        # Các node điều khiển (khởi động có trễ)
        delayed_backstepping,
        delayed_arm_dynamics,
        delayed_lstm,
    ])