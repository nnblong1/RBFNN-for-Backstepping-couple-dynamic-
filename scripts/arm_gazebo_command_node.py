#!/usr/bin/env python3
"""
arm_gazebo_command_node.py
--------------------------
Nhận lệnh JointState từ /arm_controller/joint_trajectory_plan
và publish trực tiếp tới Gazebo transport (KHÔNG cần ros_gz_bridge).

Cách tiếp cận:
  1. Thử dùng gz-transport Python bindings (nhanh, không overhead)
  2. Nếu không có, dùng subprocess gọi `gz topic -p` (chậm hơn nhưng luôn hoạt động)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import shutil

# ── Thử import gz-transport Python bindings ──
GZ_TRANSPORT_OK = False
GzNode = None
GzDouble = None

for _transport, _msgs in [
    ('gz.transport13', 'gz.msgs10'),
    ('gz.transport12', 'gz.msgs9'),
]:
    try:
        _t = __import__(_transport, fromlist=['Node'])
        _m = __import__(f'{_msgs}.double_pb2', fromlist=['Double'])
        GzNode = _t.Node
        GzDouble = _m.Double
        GZ_TRANSPORT_OK = True
        break
    except ImportError:
        continue


class ArmGazeboCommandNode(Node):
    """Chuyển đổi JointState → publish trực tiếp tới Gazebo transport."""

    JOINT_NAMES = [
        'Joint_1', 'Joint_2', 'Joint_3',
        'Joint_4', 'Joint_5', 'Joint_6',
    ]

    def __init__(self):
        super().__init__('arm_gazebo_command_node')

        # Prefix cho Gazebo topic
        self.declare_parameter('model_prefix', 'model/robot_arm')
        prefix = self.get_parameter('model_prefix') \
                     .get_parameter_value().string_value

        # Build danh sách topic Gazebo
        self.gz_topics = []
        for jname in self.JOINT_NAMES:
            topic = f'/{prefix}/joint/{jname}/0/cmd_pos'
            self.gz_topics.append(topic)

        # ── Chọn phương thức publish ──
        if GZ_TRANSPORT_OK:
            self._init_gz_transport()
        else:
            self._init_subprocess_fallback()

        # Subscribe lệnh trajectory plan (ROS 2)
        self.create_subscription(
            JointState,
            '/arm_controller/joint_trajectory_plan',
            self._on_joint_plan,
            10,
        )

        self.get_logger().info(
            f'Arm Gazebo Command Node started | '
            f'method={"gz-transport" if GZ_TRANSPORT_OK else "subprocess"}'
        )
        for t in self.gz_topics:
            self.get_logger().info(f'  → {t}')

    # ──────────────── gz-transport Python ────────────────
    def _init_gz_transport(self):
        self._gz_node = GzNode()
        self._gz_pubs = []
        for topic in self.gz_topics:
            pub = self._gz_node.advertise(topic, GzDouble)
            self._gz_pubs.append(pub)
        self.get_logger().info('Sử dụng gz-transport Python bindings')

    def _publish_gz_transport(self, positions):
        for i, pub in enumerate(self._gz_pubs):
            msg = GzDouble()
            msg.data = float(positions[i])
            pub.publish(msg)

    # ──────────────── subprocess fallback ────────────────
    def _init_subprocess_fallback(self):
        self._gz_bin = shutil.which('gz') or shutil.which('ign')
        if not self._gz_bin:
            self.get_logger().error(
                'Không tìm thấy lệnh gz hoặc ign! '
                'Cánh tay sẽ KHÔNG DI CHUYỂN được.'
            )
        else:
            self.get_logger().warn(
                f'gz-transport Python không có, dùng subprocess: {self._gz_bin}'
            )

    def _publish_subprocess(self, positions):
        if not self._gz_bin:
            return
        for i, topic in enumerate(self.gz_topics):
            try:
                subprocess.Popen(
                    [self._gz_bin, 'topic', '-t', topic,
                     '-m', 'gz.msgs.Double',
                     '-p', f'data: {positions[i]}'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as e:
                self.get_logger().error(f'gz topic -p failed: {e}')

    # ──────────────── Callback chính ────────────────
    def _on_joint_plan(self, msg: JointState):
        if len(msg.position) < 6:
            self.get_logger().warn(
                f'JointState chỉ có {len(msg.position)} phần tử, cần 6'
            )
            return

        pos = list(msg.position[:6])

        if GZ_TRANSPORT_OK:
            self._publish_gz_transport(pos)
        else:
            self._publish_subprocess(pos)

        self.get_logger().debug(
            f'Cmd: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}, '
            f'{pos[3]:.3f}, {pos[4]:.3f}, {pos[5]:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmGazeboCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
