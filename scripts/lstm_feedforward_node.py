#!/usr/bin/env python3
"""
lstm_feedforward_node.py
------------------------
Node Python chạy mô hình LSTM để dự báo mô-men xoắn nhiễu trước khi
cánh tay robot 6-DoF tạo ra sự dịch chuyển trọng tâm thực tế.

Luồng dữ liệu:
  /arm_controller/joint_trajectory_plan  →  LSTM Model  →  /ai/lstm_predictive_torque

Tần số suy luận: ~20 Hz (chạy trên CPU Raspberry Pi 4)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

import torch
import torch.nn as nn
import numpy as np
from collections import deque
import os
import time


# ============================================================
#  Định nghĩa kiến trúc Mạng LSTM
# ============================================================
class LSTMTrajectoryPredictor(nn.Module):
    """
    Mạng LSTM 2 lớp dự báo mô-men xoắn từ chuỗi kế hoạch khớp.

    Input  : [batch, seq_len, 6]  - Chuỗi góc 6 khớp robot
    Output : [batch, 3]           - Dự báo torque [τx, τy, τz]
    """

    def __init__(self,
                 input_features: int = 6,
                 hidden_dim: int = 64,
                 num_layers: int = 2,
                 output_dim: int = 3,
                 dropout: float = 0.1):
        super().__init__()

        self.hidden_dim  = hidden_dim
        self.num_layers  = num_layers

        # Lớp LSTM xử lý chuỗi thời gian kế hoạch động học
        self.lstm = nn.LSTM(
            input_size   = input_features,
            hidden_size  = hidden_dim,
            num_layers   = num_layers,
            batch_first  = True,
            dropout      = dropout if num_layers > 1 else 0.0
        )

        # Chuẩn hóa lớp để huấn luyện ổn định hơn
        self.layer_norm = nn.LayerNorm(hidden_dim)

        # Lớp kết nối đầy đủ: ánh xạ không gian ẩn → 3 mô-men xoắn
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 32),
            nn.Tanh(),
            nn.Linear(32, output_dim)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        x: [batch, seq_len, input_features]
        """
        lstm_out, _ = self.lstm(x)          # [batch, seq_len, hidden_dim]
        last_out    = lstm_out[:, -1, :]    # Lấy time step cuối
        normed      = self.layer_norm(last_out)
        return self.fc(normed)              # [batch, output_dim]

    def init_hidden(self, batch_size: int = 1):
        """Khởi tạo hidden state bằng 0"""
        device = next(self.parameters()).device
        h0 = torch.zeros(self.num_layers, batch_size, self.hidden_dim).to(device)
        c0 = torch.zeros(self.num_layers, batch_size, self.hidden_dim).to(device)
        return h0, c0


# ============================================================
#  Node ROS2: LSTM Predictive Feedforward
# ============================================================
class LSTMPredictiveNode(Node):

    # Cấu hình mặc định
    SEQ_LEN        = 10    # Chiều dài chuỗi đầu vào (time steps)
    INFER_FREQ_HZ  = 20    # Tần số suy luận
    MODEL_PATH     = os.path.join(
                        os.path.dirname(__file__),
                        '../models/lstm_uam_weights.pth'
                     )

    def __init__(self):
        super().__init__('lstm_predictive_node')

        # ── Tham số có thể cấu hình ──
        self.declare_parameter('seq_len',       self.SEQ_LEN)
        self.declare_parameter('model_path',    self.MODEL_PATH)
        self.declare_parameter('scale_factor',  1.0)   # Nhân hệ số khuếch đại output
        self.declare_parameter('enable_filter', True)  # Bộ lọc thông thấp đơn giản

        seq_len      = self.get_parameter('seq_len').value
        model_path   = self.get_parameter('model_path').value
        self.scale   = self.get_parameter('scale_factor').value
        self.use_lpf = self.get_parameter('enable_filter').value

        # ── Thiết bị tính toán ──
        self.device = torch.device('cpu')  # RPi 4 không có GPU

        # ── Tải mô hình LSTM ──
        self.model = LSTMTrajectoryPredictor(
            input_features=6,
            hidden_dim=64,
            num_layers=2,
            output_dim=3
        ).to(self.device)

        if os.path.exists(model_path):
            state_dict = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(state_dict)
            self.get_logger().info(f"Đã tải trọng số LSTM từ: {model_path}")
        else:
            self.get_logger().warn(
                f"Không tìm thấy file trọng số '{model_path}' – "
                "Chạy với trọng số ngẫu nhiên (CHỈ DÙNG CHO TEST)."
            )

        self.model.eval()
        torch.set_num_threads(2)  # Giới hạn CPU threads để không ảnh hưởng node C++

        # ── Bộ đệm chuỗi trượt (Sliding Window Buffer) ──
        self.seq_buffer: deque = deque(maxlen=seq_len)
        self.seq_len = seq_len

        # ── Bộ lọc thông thấp đơn giản (Exponential Moving Average) ──
        self.lpf_alpha   = 0.3              # α nhỏ = lọc mạnh hơn
        self.lpf_output  = np.zeros(3)

        # ── Trạng thái suy luận ──
        self.last_prediction = np.zeros(3)
        self.infer_count     = 0
        self.total_infer_ms  = 0.0

        # ── Publisher ──
        self.ff_pub = self.create_publisher(
            Vector3,
            '/ai/lstm_predictive_torque',
            10
        )

        # ── QoS tốt nhất ──
        best_effort_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10
        )

        # ── Subscriber nhận kế hoạch quỹ đạo khớp từ MoveIt2 ──
        self.plan_sub = self.create_subscription(
            JointState,
            '/arm_controller/joint_trajectory_plan',
            self.trajectory_callback,
            best_effort_qos
        )

        # ── Timer suy luận định kỳ (20 Hz) ──
        period_ms = int(1000 / self.INFER_FREQ_HZ)
        self.infer_timer = self.create_timer(
            period_ms / 1000.0,
            self.inference_loop
        )

        # ── Timer báo cáo hiệu năng (mỗi 10 giây) ──
        self.perf_timer = self.create_timer(10.0, self.log_performance)

        self.get_logger().info(
            f"LSTM Feedforward Node sẵn sàng | "
            f"seq_len={seq_len} | infer={self.INFER_FREQ_HZ}Hz"
        )

    # ────────────────────────────────────────────────────────
    def trajectory_callback(self, msg: JointState):
        """
        Nhận trạng thái khớp từ topic kế hoạch, cập nhật buffer trượt.
        """
        if len(msg.position) < 6:
            return

        joints = np.array(msg.position[:6], dtype=np.float32)
        self.seq_buffer.append(joints)

    # ────────────────────────────────────────────────────────
    def inference_loop(self):
        """
        Chạy suy luận LSTM mỗi 50ms (20 Hz) nếu buffer đầy đủ.
        """
        if len(self.seq_buffer) < self.seq_len:
            # Chưa đủ dữ liệu → phát tín hiệu bù tiến = 0
            self._publish_torque(np.zeros(3))
            return

        # Chuyển buffer thành Tensor [1, seq_len, 6]
        seq_array  = np.array(list(self.seq_buffer), dtype=np.float32)
        seq_tensor = torch.tensor(seq_array).unsqueeze(0).to(self.device)

        # ── Suy luận LSTM ──
        t_start = time.perf_counter()
        with torch.no_grad():
            raw_pred = self.model(seq_tensor).squeeze(0).cpu().numpy()
        infer_ms = (time.perf_counter() - t_start) * 1000.0

        # Cập nhật thống kê hiệu năng
        self.infer_count     += 1
        self.total_infer_ms  += infer_ms

        # Áp tỷ lệ khuếch đại
        raw_pred = raw_pred * self.scale

        # Bộ lọc EMA (Exponential Moving Average) để làm mịn tín hiệu
        if self.use_lpf:
            self.lpf_output = (self.lpf_alpha * raw_pred
                               + (1.0 - self.lpf_alpha) * self.lpf_output)
            output = self.lpf_output
        else:
            output = raw_pred

        self.last_prediction = output
        self._publish_torque(output)

    # ────────────────────────────────────────────────────────
    def _publish_torque(self, torque: np.ndarray):
        msg = Vector3()
        msg.x = float(torque[0])  # τ_x (Roll)
        msg.y = float(torque[1])  # τ_y (Pitch)
        msg.z = float(torque[2])  # τ_z (Yaw)
        self.ff_pub.publish(msg)

    # ────────────────────────────────────────────────────────
    def log_performance(self):
        if self.infer_count > 0:
            avg_ms = self.total_infer_ms / self.infer_count
            self.get_logger().info(
                f"[LSTM Perf] Suy luận trung bình: {avg_ms:.2f}ms | "
                f"Tổng: {self.infer_count} lần | "
                f"Dự báo: [{self.last_prediction[0]:.4f}, "
                f"{self.last_prediction[1]:.4f}, "
                f"{self.last_prediction[2]:.4f}] N·m"
            )


# ============================================================
#  Entry point
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = LSTMPredictiveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
