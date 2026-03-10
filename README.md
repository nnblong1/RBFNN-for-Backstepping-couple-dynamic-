# UAM Adaptive Backstepping + AI Controller

**Khung điều khiển Backstepping Thích nghi tích hợp RBFNN + LSTM**  
cho Hệ thống Máy bay Không người lái mang Cánh tay Robot 6 Bậc tự do

---

## Kiến trúc Hệ thống

```
┌──────────────────────────────────────────────────────────────────┐
│                        Raspberry Pi 4                            │
│                                                                  │
│  ┌─────────────────────────┐   ┌──────────────────────────────┐  │
│  │  C++ Node (100 Hz)      │   │  Python Node (20 Hz)         │  │
│  │                         │   │                              │  │
│  │  Backstepping (2 vòng)  │◄──│  LSTM Predictive Feedforward │  │
│  │    + RBFNN Online Learn │   │  (Dự báo nhiễu từ kế hoạch   │  │
│  │    + Lyapunov Stability │   │   quỹ đạo cánh tay)          │  │
│  └────────────┬────────────┘   └──────────────────────────────┘  │
│               │ τ, T (Torque + Thrust)                           │
│  ┌────────────▼──────────────────────────────────────────────┐   │
│  │           Micro XRCE-DDS Bridge (ROS2 ↔ PX4)             │   │
│  └────────────────────────────────┬──────────────────────────┘   │
└───────────────────────────────────│──────────────────────────────┘
                                    │ UART 921600 bps
                           ┌────────▼────────┐
                           │  PX4 Autopilot  │
                           │  Offboard Mode  │
                           │  direct_actuator│
                           └────────┬────────┘
                                    │ ESC PWM
                           ┌────────▼────────┐
                           │   4× Rotor ESC  │
                           └─────────────────┘
```

---

## Cấu trúc Thư mục

```
uam_controller/
├── include/uam_controller/
│   └── uam_adaptive_controller.hpp    # Header định nghĩa lớp + cấu trúc tham số
├── src/
│   └── uam_backstepping_rbfnn_node.cpp  # Lõi điều khiển C++ (100 Hz)
├── scripts/
│   ├── lstm_feedforward_node.py       # Node AI dự báo (20 Hz)
│   ├── arm_dynamics_node.py           # Newton-Euler RNE (50 Hz)
│   └── train_lstm_offline.py          # Script huấn luyện offline
├── launch/
│   └── uam_system.launch.py           # Khởi động toàn bộ hệ thống
├── models/
│   └── lstm_uam_weights.pth           # Trọng số LSTM (sau khi train)
├── config/
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Phụ thuộc Hệ thống

### Phần mềm
| Thành phần | Phiên bản |
|-----------|-----------|
| Ubuntu    | 22.04 LTS (64-bit ARM) |
| ROS2      | Humble Hawksbill |
| PX4 Autopilot | v1.14+ |
| Micro XRCE-DDS Agent | 2.4+ |
| PyTorch (CPU) | 2.0+ |
| Eigen3    | 3.4+ |

### Phần cứng
- Raspberry Pi 4 (4GB RAM)
- PX4 Flight Controller (Pixhawk 4 hoặc tương đương)
- Kết nối UART: `/dev/ttyAMA0` @ 921600 bps

---

## Cài đặt

### 1. Cài đặt phụ thuộc Python
```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install numpy pandas matplotlib
```

### 2. Clone px4_msgs và build workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone <repo_này> uam_controller
cd ~/ros2_ws
colcon build --packages-select px4_msgs uam_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3. Cấu hình PX4 Firmware
Thêm topics vào `dds_topics.yaml` trong PX4 source:
```yaml
publications:
  - topic: /fmu/out/vehicle_odometry
    type: px4_msgs::msg::VehicleOdometry
  - topic: /fmu/out/vehicle_status
    type: px4_msgs::msg::VehicleStatus

subscriptions:
  - topic: /fmu/in/offboard_control_mode
    type: px4_msgs::msg::OffboardControlMode
  - topic: /fmu/in/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint
  - topic: /fmu/in/vehicle_thrust_setpoint
    type: px4_msgs::msg::VehicleThrustSetpoint
```

---

## Huấn luyện LSTM (Offline)

```bash
# Tạo dataset tổng hợp + huấn luyện (nếu chưa có Gazebo data):
cd ~/ros2_ws/src/uam_controller/scripts
python3 train_lstm_offline.py \
  --epochs 150 \
  --batch_size 128 \
  --lr 0.001 \
  --seq_len 10 \
  --model_out ../models/lstm_uam_weights.pth

# Với dữ liệu Gazebo thực tế:
python3 train_lstm_offline.py \
  --data_path /path/to/gazebo_recording.csv \
  --epochs 200
```

---

## Khởi động Hệ thống

### Môi trường mô phỏng (Gazebo SITL)
```bash
# Terminal 1: Khởi động PX4 SITL + Gazebo
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris

# Terminal 2: Khởi động toàn bộ stack ROS2
ros2 launch uam_controller uam_system.launch.py sim:=true

# Terminal 3: Gửi điểm đặt vị trí
ros2 topic pub /uam/position_setpoint geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 0.0, z: 2.0}"
```

### Phần cứng thực tế
```bash
ros2 launch uam_controller uam_system.launch.py sim:=false
```

---

## Giám sát và Debug

```bash
# Xem trạng thái debug (vị trí, tư thế, sai số, RBFNN output)
ros2 topic echo /uam/debug_state

# Xem tín hiệu bù tiến LSTM
ros2 topic echo /ai/lstm_predictive_torque

# Xem tương tác lực cánh tay
ros2 topic echo /arm/interaction_wrench

# Tần số vòng lặp điều khiển
ros2 topic hz /fmu/in/vehicle_torque_setpoint
```

---

## Điều chỉnh Tham số (Tuning)

| Tham số | Ý nghĩa | Tăng nếu | Giảm nếu |
|---------|---------|----------|----------|
| `K1_pos` | Độ cứng vị trí | Đáp ứng chậm | Dao động vị trí |
| `K2_vel` | Tắt dần vận tốc | Vọt lố vị trí | Phản hồi chậm |
| `K1_att` | Độ cứng tư thế | Tư thế chậm | Rung lắc tư thế |
| `K2_rate` | Tắt dần tốc độ góc | Vọt lố góc | Đáp ứng góc chậm |
| `rbfnn_lr` | Tốc độ học RBFNN | Hội tụ chậm | Dao động trọng số |
| `rbfnn_eta` | E-modification | Phân kỳ trọng số | Quá trơn ước lượng |

---

## Thuật toán Cốt lõi

### Luật Điều khiển Tổng hợp

$$U_{total} = G^\dagger(X)\left[-e_1 - \hat{F}(Z) - K_2 e_2 - \dot{\alpha}\right] + \hat{\tau}_{LSTM}$$

### Cập nhật Trọng số RBFNN (Lyapunov-based)

$$\dot{\hat{W}} = \Gamma\left[h(Z)e_2^T - \eta\|e_2\|\hat{W}\right]$$

### Hàm Lyapunov Tổng thể

$$V_3 = \frac{1}{2}e_1^Te_1 + \frac{1}{2}e_2^Te_2 + \frac{1}{2}\text{tr}(\tilde{W}^T\Gamma^{-1}\tilde{W})$$

$$\dot{V}_3 \leq -e_1^TK_1e_1 - e_2^TK_2e_2 + e_2^T\epsilon \leq 0 \quad \text{(SUGL)}$$

---

## Giấy phép

MIT License — Tự do sử dụng cho nghiên cứu và ứng dụng thực tế.
