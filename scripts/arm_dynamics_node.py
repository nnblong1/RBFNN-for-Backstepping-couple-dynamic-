#!/usr/bin/env python3
"""
arm_dynamics_node.py
--------------------
Node Python tính toán lực và mô-men xoắn tương tác từ cánh tay 6-DoF
lên thân UAV bằng thuật toán đệ quy Newton-Euler (RNE).

Kết quả được xuất sang /arm/interaction_wrench để node C++ tham khảo
và so sánh với đầu ra RBFNN (giám sát chất lượng học thích nghi).

Tần số: 50 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import numpy as np
from typing import List, Optional


# ============================================================
#  Cấu trúc thông số hình học một Link (Denavit-Hartenberg)
# ============================================================
class DHLink:
    """
    Thông số Denavit-Hartenberg chuẩn cho một khớp quay.
    """
    def __init__(self,
                 alpha: float, a: float,
                 d: float, mass: float,
                 com: np.ndarray,
                 inertia: np.ndarray):
        """
        alpha : góc xoắn DH [rad]
        a     : khoảng cách DH [m]
        d     : dịch chuyển DH [m]
        mass  : khối lượng link [kg]
        com   : trọng tâm link trong frame link [m], shape (3,)
        inertia: tensor quán tính 3×3 tại CoM [kg·m²]
        """
        self.alpha   = alpha
        self.a       = a
        self.d       = d
        self.mass    = mass
        self.com     = np.array(com).reshape(3)
        self.inertia = np.array(inertia).reshape(3, 3)


# ============================================================
#  Lớp tính toán đệ quy Newton-Euler
# ============================================================
class RecursiveNewtonEuler:
    """
    Thuật toán đệ quy Newton-Euler tính lực/mô-men tương tác.

    Tham chiếu: Siciliano et al., "Robotics: Modelling, Planning and Control"
    """

    GRAVITY = np.array([0.0, 0.0, -9.81])  # Trọng lực trong inertial frame

    def __init__(self, links: List[DHLink]):
        self.links  = links
        self.n_dof  = len(links)

    def dh_rotation(self, alpha: float, theta: float) -> np.ndarray:
        """Ma trận xoay DH: R_i-1_i"""
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ct,        -st,       0.0],
            [st * ca,    ct * ca,  -sa],
            [st * sa,    ct * sa,   ca]
        ])

    def dh_transform(self, alpha: float, a: float,
                      d: float, theta: float) -> np.ndarray:
        """Ma trận biến đổi đồng nhất 4×4 DH"""
        R = self.dh_rotation(alpha, theta)
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3]   = a
        T[1, 3]   = -d * np.sin(alpha)
        T[2, 3]   =  d * np.cos(alpha)
        return T

    def compute_interaction_wrench(self,
                                    q:  np.ndarray,
                                    dq: np.ndarray,
                                    ddq: np.ndarray,
                                    base_acc: Optional[np.ndarray] = None
                                    ) -> np.ndarray:
        """
        Tính lực và mô-men tại khớp 0 (điểm gắn vào UAV).

        Tham số:
            q    : góc khớp [rad]   shape (n,)
            dq   : vận tốc khớp     shape (n,)
            ddq  : gia tốc khớp     shape (n,)
            base_acc : gia tốc thân UAV [m/s²], mặc định = trọng lực

        Trả về:
            wrench_0 : [fx, fy, fz, nx, ny, nz] tại khớp 0
        """
        n = self.n_dof
        if base_acc is None:
            base_acc = -self.GRAVITY  # gia tốc trọng trường lên trên

        # ── Đệ quy tiến (Forward Recursion) ──
        # Vận tốc góc ω, gia tốc góc α, gia tốc tuyến tính tại từng link
        omega_prev  = np.zeros(3)        # ω_0 = 0
        alpha_prev  = np.zeros(3)        # α_0 = 0
        a_prev      = base_acc.copy()    # gia tốc tại gốc

        omegas  = []
        alphas  = []
        a_links = []
        a_coms  = []
        R_mats  = []

        for i in range(n):
            lnk = self.links[i]
            R   = self.dh_rotation(lnk.alpha, q[i])   # R_i-1_i
            R_T = R.T                                   # R_i_i-1

            z_prev = np.array([0.0, 0.0, 1.0])          # trục z trong frame i-1

            # Vận tốc góc: ω_i = R^T * ω_i-1 + dq_i * z_i
            omega_i = R_T @ omega_prev + dq[i] * np.array([0, 0, 1])

            # Gia tốc góc: α_i = R^T * α_i-1 + ddq_i * z_i + ω_i × (dq_i * z_i)
            alpha_i = (R_T @ alpha_prev
                       + ddq[i] * np.array([0, 0, 1])
                       + np.cross(omega_i, dq[i] * np.array([0, 0, 1])))

            # Vector vị trí link trong frame i
            p_i = np.array([lnk.a, -lnk.d * np.sin(lnk.alpha),
                             lnk.d * np.cos(lnk.alpha)])

            # Gia tốc tuyến tính origin of frame i
            a_i = (R_T @ a_prev
                   + np.cross(alpha_i, p_i)
                   + np.cross(omega_i, np.cross(omega_i, p_i)))

            # Gia tốc tuyến tính tại CoM
            a_com_i = (a_i
                       + np.cross(alpha_i, lnk.com)
                       + np.cross(omega_i, np.cross(omega_i, lnk.com)))

            omegas.append(omega_i)
            alphas.append(alpha_i)
            a_links.append(a_i)
            a_coms.append(a_com_i)
            R_mats.append(R)

            omega_prev = omega_i
            alpha_prev = alpha_i
            a_prev     = a_i

        # ── Đệ quy lùi (Backward Recursion) ──
        # Khởi tạo lực và mô-men tại end-effector = 0 (không tải trọng)
        f_next = np.zeros(3)
        n_next = np.zeros(3)

        for i in range(n - 1, -1, -1):
            lnk = self.links[i]
            R   = R_mats[i]

            # Lực quán tính Newton
            F_i = lnk.mass * a_coms[i]

            # Mô-men quán tính Euler
            N_i = (lnk.inertia @ alphas[i]
                   + np.cross(omegas[i], lnk.inertia @ omegas[i]))

            # Vector vị trí link trong frame i
            p_i = np.array([lnk.a, -lnk.d * np.sin(lnk.alpha),
                             lnk.d * np.cos(lnk.alpha)])

            # Đệ quy lùi lực: f_i = R_i+1 * f_i+1 + F_i
            if i < n - 1:
                R_next = R_mats[i + 1]
                f_i    = R_next @ f_next + F_i
            else:
                f_i    = f_next + F_i

            # Đệ quy lùi mô-men: n_i = R_i+1*n_i+1 + p_i × f_i + com_i × F_i + N_i
            if i < n - 1:
                R_next = R_mats[i + 1]
                n_i    = (R_next @ n_next
                          + np.cross(p_i, f_i)
                          + np.cross(lnk.com, F_i)
                          + N_i)
            else:
                n_i    = (np.cross(p_i, f_i)
                          + np.cross(lnk.com, F_i)
                          + N_i)

            f_next = f_i
            n_next = n_i

        # f_next, n_next tại khớp 0 = tương tác lên UAV
        return np.concatenate([f_next, n_next])


# ============================================================
#  Node ROS2
# ============================================================
class ArmDynamicsNode(Node):

    def __init__(self):
        super().__init__('arm_dynamics_node')

        # ── Cấu hình cánh tay 6-DoF (tham số UR5-like) ──
        # Đây là ví dụ với cánh tay 6 khớp quay, cần thay bằng DH thực tế
        links = [
            DHLink(alpha= 0.0,           a=0.0,   d=0.089, mass=3.7,
                   com=[0.0, 0.0, 0.044],
                   inertia=np.diag([0.0071, 0.0071, 0.0033])),
            DHLink(alpha= np.pi/2,       a=0.0,   d=0.0,   mass=8.4,
                   com=[0.0, 0.045, 0.135],
                   inertia=np.diag([0.022, 0.022, 0.009])),
            DHLink(alpha= 0.0,           a=0.425, d=0.0,   mass=2.3,
                   com=[0.0, 0.012, 0.106],
                   inertia=np.diag([0.0050, 0.0050, 0.0020])),
            DHLink(alpha= 0.0,           a=0.392, d=0.109, mass=1.2,
                   com=[0.0, -0.012, 0.073],
                   inertia=np.diag([0.0010, 0.0010, 0.0004])),
            DHLink(alpha= np.pi/2,       a=0.0,   d=0.095, mass=1.2,
                   com=[0.0, 0.0, 0.055],
                   inertia=np.diag([0.0010, 0.0010, 0.0004])),
            DHLink(alpha=-np.pi/2,       a=0.0,   d=0.082, mass=0.2,
                   com=[0.0, 0.0, 0.012],
                   inertia=np.diag([0.0001, 0.0001, 0.0001]))
        ]

        self.rne = RecursiveNewtonEuler(links)

        # Trạng thái khớp hiện tại
        self.q   = np.zeros(6)
        self.dq  = np.zeros(6)
        self.ddq = np.zeros(6)
        self._prev_dq   = np.zeros(6)
        self._prev_time = None

        # ── Publisher ──
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/arm/interaction_wrench',
            10
        )

        # ── Subscriber ──
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info("Arm Dynamics (Newton-Euler) Node khởi động.")

    def joint_callback(self, msg: JointState):
        if len(msg.position) < 6:
            return

        now = self.get_clock()->now().seconds() if hasattr(self, '_clock') \
              else rclpy.clock.Clock().now().nanoseconds() * 1e-9

        self.q[:] = list(msg.position[:6])

        if msg.velocity and len(msg.velocity) >= 6:
            self.dq[:] = list(msg.velocity[:6])
        elif self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                self.dq[:] = (self.q - self._prev_q) / dt

        if self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0:
                self.ddq[:] = (self.dq - self._prev_dq) / dt

        self._prev_dq   = self.dq.copy()
        self._prev_q    = self.q.copy()
        self._prev_time = now

        # Tính wrench tương tác
        wrench = self.rne.compute_interaction_wrench(self.q, self.dq, self.ddq)

        msg_out = WrenchStamped()
        msg_out.header.stamp    = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "base_link"
        msg_out.wrench.force.x  = float(wrench[0])
        msg_out.wrench.force.y  = float(wrench[1])
        msg_out.wrench.force.z  = float(wrench[2])
        msg_out.wrench.torque.x = float(wrench[3])
        msg_out.wrench.torque.y = float(wrench[4])
        msg_out.wrench.torque.z = float(wrench[5])
        self.wrench_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmDynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
