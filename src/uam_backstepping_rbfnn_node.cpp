/**
 * @file uam_backstepping_rbfnn_node.cpp
 * @brief Node C++ điều khiển UAM bằng Adaptive Backstepping + RBFNN
 *
 * Kiến trúc:
 *   - Vòng ngoài (Outer Loop) : Backstepping vị trí → tạo tư thế tham chiếu
 *   - Vòng trong (Inner Loop) : Backstepping tư thế + RBFNN bù nhiễu
 *   - Tích hợp tín hiệu bù tiến LSTM từ Python node
 *
 * Tần số: 100 Hz (10 ms)
 * Giao tiếp PX4: Micro XRCE-DDS | Offboard direct-actuator mode
 */

#include "uam_controller/uam_adaptive_controller.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

// ============================================================
//  Triển khai lớp RBFNeuralNetwork
// ============================================================

RBFNeuralNetwork::RBFNeuralNetwork(const RBFNNParams& params)
    : params_(params)
{
    // Khởi tạo ma trận trọng số W_hat = 0
    W_hat_ = Eigen::MatrixXd::Zero(params_.num_neurons, params_.output_dim);

    // Phân bố đều các trung tâm Gaussian trong không gian đầu vào [-π, π]
    C_centers_ = Eigen::MatrixXd::Random(params_.num_neurons, params_.input_dim);
    C_centers_ *= M_PI; // Scale về [-π, π]

    // Tất cả nơ-ron cùng độ rộng Gaussian
    B_widths_ = Eigen::VectorXd::Constant(params_.num_neurons, params_.gaussian_width);

    // Ma trận tốc độ học Γ (scalar * I)
    Gamma_ = Eigen::MatrixXd::Identity(params_.num_neurons, params_.num_neurons)
             * params_.learning_rate;
}

Eigen::VectorXd RBFNeuralNetwork::compute_basis(const Eigen::VectorXd& Z) const
{
    Eigen::VectorXd h(params_.num_neurons);
    for (int i = 0; i < params_.num_neurons; ++i) {
        Eigen::VectorXd diff = Z - C_centers_.row(i).transpose();
        double norm_sq = diff.squaredNorm();
        h(i) = std::exp(-norm_sq / (2.0 * B_widths_(i) * B_widths_(i)));
    }
    return h;
}

Eigen::VectorXd RBFNeuralNetwork::estimate(const Eigen::VectorXd& Z) const
{
    Eigen::VectorXd h = compute_basis(Z);
    return W_hat_.transpose() * h;  // F_hat = W_hat^T * h(Z)
}

void RBFNeuralNetwork::update_weights(const Eigen::VectorXd& Z,
                                       const Eigen::VectorXd& e2,
                                       double dt)
{
    Eigen::VectorXd h = compute_basis(Z);

    // Luật cập nhật Lyapunov-based:
    // dW_hat = Γ * (h * e2^T - η * ||e2|| * W_hat)
    double e2_norm = e2.norm();
    Eigen::MatrixXd dW = Gamma_ * (h * e2.transpose()
                         - params_.e_modification * e2_norm * W_hat_);

    // Tích phân Euler: W_hat(k+1) = W_hat(k) + dW * dt
    W_hat_ += dW * dt;

    // Hard clipping để đảm bảo giới hạn trọng số (tăng cường an toàn)
    W_hat_ = W_hat_.cwiseMin(50.0).cwiseMax(-50.0);
}

void RBFNeuralNetwork::reset()
{
    W_hat_.setZero();
}


// ============================================================
//  Triển khai UAMAdaptiveController
// ============================================================

UAMAdaptiveController::UAMAdaptiveController()
    : Node("uam_adaptive_controller")
{
    // ── Tải tham số từ ROS2 parameter server ──
    this->declare_parameter("mass_total",       sys_params_.mass_total);
    this->declare_parameter("Ixx",              sys_params_.Ixx);
    this->declare_parameter("Iyy",              sys_params_.Iyy);
    this->declare_parameter("Izz",              sys_params_.Izz);
    this->declare_parameter("K1_pos",           bs_params_.K1_pos);
    this->declare_parameter("K2_vel",           bs_params_.K2_vel);
    this->declare_parameter("K1_att",           bs_params_.K1_att);
    this->declare_parameter("K2_rate",          bs_params_.K2_rate);
    this->declare_parameter("rbfnn_neurons",    rbfnn_params_.num_neurons);
    this->declare_parameter("rbfnn_lr",         rbfnn_params_.learning_rate);
    this->declare_parameter("rbfnn_eta",        rbfnn_params_.e_modification);
    this->declare_parameter("rbfnn_width",      rbfnn_params_.gaussian_width);

    sys_params_.mass_total      = this->get_parameter("mass_total").as_double();
    sys_params_.Ixx             = this->get_parameter("Ixx").as_double();
    sys_params_.Iyy             = this->get_parameter("Iyy").as_double();
    sys_params_.Izz             = this->get_parameter("Izz").as_double();
    bs_params_.K1_pos           = this->get_parameter("K1_pos").as_double();
    bs_params_.K2_vel           = this->get_parameter("K2_vel").as_double();
    bs_params_.K1_att           = this->get_parameter("K1_att").as_double();
    bs_params_.K2_rate          = this->get_parameter("K2_rate").as_double();
    rbfnn_params_.num_neurons   = this->get_parameter("rbfnn_neurons").as_int();
    rbfnn_params_.learning_rate = this->get_parameter("rbfnn_lr").as_double();
    rbfnn_params_.e_modification= this->get_parameter("rbfnn_eta").as_double();
    rbfnn_params_.gaussian_width= this->get_parameter("rbfnn_width").as_double();

    // ── Khởi tạo RBFNN ──
    rbfnn_ = std::make_unique<RBFNeuralNetwork>(rbfnn_params_);

    // ── Khởi tạo trạng thái ──
    position_.setZero();
    velocity_.setZero();
    euler_angles_.setZero();
    angular_rate_.setZero();
    lstm_ff_torque_.setZero();
    position_desired_ = Eigen::Vector3d(0.0, 0.0, 1.5); // Hover tại z = 1.5m
    euler_desired_.setZero();
    yaw_desired_ = 0.0;
    total_thrust_ = sys_params_.mass_total * sys_params_.gravity; // Thrust cân bằng trọng lực

    // ── Publishers ──
    auto qos_reliable = rclcpp::QoS(10).reliable();
    offboard_mode_pub_  = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                            "/fmu/in/offboard_control_mode", qos_reliable);
    torque_setpoint_pub_= this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
                            "/fmu/in/vehicle_torque_setpoint", qos_reliable);
    thrust_setpoint_pub_= this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
                            "/fmu/in/vehicle_thrust_setpoint", qos_reliable);
    debug_pub_          = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                            "/uam/debug_state", 10);

    // ── Subscribers ──
    auto qos_best_effort = rclcpp::QoS(10).best_effort();
    odom_sub_    = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                    "/fmu/out/vehicle_odometry", qos_best_effort,
                    std::bind(&UAMAdaptiveController::odometry_callback,
                              this, std::placeholders::_1));

    status_sub_  = this->create_subscription<px4_msgs::msg::VehicleStatus>(
                    "/fmu/out/vehicle_status", qos_best_effort,
                    std::bind(&UAMAdaptiveController::status_callback,
                              this, std::placeholders::_1));

    lstm_sub_    = this->create_subscription<geometry_msgs::msg::Vector3>(
                    "/ai/lstm_predictive_torque", 10,
                    std::bind(&UAMAdaptiveController::lstm_callback,
                              this, std::placeholders::_1));

    setpoint_sub_= this->create_subscription<geometry_msgs::msg::Vector3>(
                    "/uam/position_setpoint", 10,
                    std::bind(&UAMAdaptiveController::setpoint_callback,
                              this, std::placeholders::_1));

    // ── Timer vòng lặp điều khiển 100 Hz ──
    control_timer_ = this->create_wall_timer(
                        10ms,
                        std::bind(&UAMAdaptiveController::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "UAM Adaptive Backstepping + RBFNN Controller khởi động | 100 Hz | %d neurons",
                rbfnn_params_.num_neurons);
}


// ============================================================
//  Callback nhận odometry từ PX4
// ============================================================
void UAMAdaptiveController::odometry_callback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // Vị trí: NED → ENU conversion (PX4 dùng NED)
    position_ << msg->position[0],
                -msg->position[1],
                -msg->position[2];

    // Vận tốc tuyến tính
    velocity_ << msg->velocity[0],
                -msg->velocity[1],
                -msg->velocity[2];

    // Quaternion → Euler
    quaternion_ = Eigen::Quaterniond(msg->q[0], msg->q[1],
                                      msg->q[2], msg->q[3]);
    euler_angles_ = quaternion_to_euler(quaternion_);

    // Tốc độ góc trong Body Frame
    angular_rate_ << msg->angular_velocity[0],
                    -msg->angular_velocity[1],
                    -msg->angular_velocity[2];

    has_odometry_ = true;
}

void UAMAdaptiveController::status_callback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    // nav_state == 14 tương ứng chế độ Offboard trong PX4
    is_offboard_mode_ = (msg->nav_state == 14);
}

void UAMAdaptiveController::lstm_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // Cập nhật tín hiệu bù tiến từ LSTM (feedforward torque)
    lstm_ff_torque_ << msg->x, msg->y, msg->z;
}

void UAMAdaptiveController::setpoint_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    position_desired_ << msg->x, msg->y, msg->z;
}


// ============================================================
//  Hàm phát tín hiệu Offboard heartbeat
// ============================================================
void UAMAdaptiveController::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode mode_msg{};
    mode_msg.position       = false;
    mode_msg.velocity       = false;
    mode_msg.acceleration   = false;
    mode_msg.attitude       = false;
    mode_msg.body_rate      = false;
    mode_msg.actuator       = false;
    mode_msg.direct_actuator= true;  // Truyền trực tiếp Torque + Thrust
    mode_msg.timestamp      = this->get_clock()->now().nanoseconds() / 1000;
    offboard_mode_pub_->publish(mode_msg);
}


// ============================================================
//  BƯỚC 1 (Outer Loop): Backstepping vị trí → tư thế tham chiếu
//  Trả về vector mô-men xoắn tư thế cần thiết
// ============================================================
double UAMAdaptiveController::compute_thrust_control()
{
    // === Bước 1: Sai số vị trí ===
    e1_pos_ = position_ - position_desired_;

    // === Tín hiệu ảo: vận tốc mong muốn ===
    // α_pos = -K1_pos * e1_pos
    virtual_vel_ = -bs_params_.K1_pos * e1_pos_;

    // === Bước 2: Sai số vận tốc ===
    e2_vel_ = velocity_ - virtual_vel_;

    // Lực đẩy tổng: bù trọng lực + dampen sai số vận tốc Z
    double thrust_raw = sys_params_.mass_total * (
                            sys_params_.gravity
                            - bs_params_.K2_vel * e2_vel_.z()
                            - bs_params_.K1_pos * e1_pos_.z() );

    // Chuẩn hóa về [0, 1] cho PX4
    double thrust_normalized = thrust_raw / sys_params_.max_thrust;
    return std::clamp(thrust_normalized, 0.0, 1.0);
}


// ============================================================
//  BƯỚC 2 (Inner Loop): Backstepping tư thế + RBFNN
//  Trả về vector mô-men xoắn τ = [τx, τy, τz]
// ============================================================
Eigen::Vector3d UAMAdaptiveController::compute_attitude_control(double dt)
{
    // === Tính tư thế mong muốn từ gia tốc vị trí mong muốn ===
    // (Giản lược: yaw hold + phi/theta nhỏ)
    double phi_des   = 0.0;
    double theta_des = (1.0 / sys_params_.gravity) *
                       (-bs_params_.K2_vel * e2_vel_.x() - bs_params_.K1_pos * e1_pos_.x());
    double psi_des   = yaw_desired_;

    Eigen::Vector3d euler_desired_computed(phi_des, theta_des, psi_des);
    euler_desired_computed(0) = saturate(euler_desired_computed(0), 0.35); // ±20°
    euler_desired_computed(1) = saturate(euler_desired_computed(1), 0.35);

    // === Bước 1 (inner): Sai số tư thế ===
    e1_att_ = euler_angles_ - euler_desired_computed;

    // Chuẩn hóa yaw error về [-π, π]
    while (e1_att_.z() >  M_PI) e1_att_.z() -= 2.0 * M_PI;
    while (e1_att_.z() < -M_PI) e1_att_.z() += 2.0 * M_PI;

    // === Tốc độ góc ảo ===
    Eigen::Vector3d omega_virtual = -bs_params_.K1_att * e1_att_;

    // === Bước 2 (inner): Sai số tốc độ góc ===
    e2_rate_ = angular_rate_ - omega_virtual;

    // === Đầu vào RBFNN: Z = [e1_att, e2_rate] ∈ ℝ⁶ ===
    Eigen::VectorXd Z_input(6);
    Z_input << e1_att_, e2_rate_;

    // === Ước lượng nhiễu phi tuyến từ RBFNN ===
    Eigen::VectorXd F_hat = rbfnn_->estimate(Z_input);

    // === Cập nhật trọng số trực tuyến (Online Learning) ===
    rbfnn_->update_weights(Z_input, e2_rate_, dt);

    // === Backstepping điều khiển: Phương trình Euler linearized ===
    // τ_x = Ixx * (ω_z*ω_y*(Iyy-Izz)/Ixx - K1_att*e1_x - K2_rate*e2_x - F_hat_x)
    double cross_x = angular_rate_.z() * angular_rate_.y() *
                     (sys_params_.Iyy - sys_params_.Izz) / sys_params_.Ixx;
    double cross_y = angular_rate_.x() * angular_rate_.z() *
                     (sys_params_.Izz - sys_params_.Ixx) / sys_params_.Iyy;
    double cross_z = angular_rate_.x() * angular_rate_.y() *
                     (sys_params_.Ixx - sys_params_.Iyy) / sys_params_.Izz;

    Eigen::Vector3d tau_bs;
    tau_bs(0) = sys_params_.Ixx * (-e1_att_(0) - bs_params_.K2_rate * e2_rate_(0)
                                    - F_hat(0) - cross_x);
    tau_bs(1) = sys_params_.Iyy * (-e1_att_(1) - bs_params_.K2_rate * e2_rate_(1)
                                    - F_hat(1) - cross_y);
    tau_bs(2) = sys_params_.Izz * (-e1_att_(2) - bs_params_.K2_rate * e2_rate_(2)
                                    - F_hat(2) - cross_z);

    // === Cộng tín hiệu bù tiến LSTM ===
    Eigen::Vector3d tau_total = tau_bs + lstm_ff_torque_;

    // === Giới hạn mô-men xoắn và chuẩn hóa ===
    for (int i = 0; i < 3; ++i) {
        tau_total(i) = saturate(tau_total(i), sys_params_.max_torque);
        tau_total(i) /= sys_params_.max_torque; // Chuẩn hóa [-1, 1]
    }

    publish_debug_info(tau_total, F_hat, total_thrust_);
    return tau_total;
}


// ============================================================
//  VÒNG LẶP ĐIỀU KHIỂN CHÍNH (100 Hz)
// ============================================================
void UAMAdaptiveController::control_loop()
{
    // Heartbeat: phải phát liên tục để PX4 không thoát Offboard
    publish_offboard_control_mode();
    ++offboard_counter_;

    if (!has_odometry_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(), 2000,
                             "Chờ dữ liệu Odometry...");
        return;
    }

    // Tính dt (chu kỳ thực tế)
    double now = this->get_clock()->now().seconds();
    double dt  = (last_time_ > 0.0) ? (now - last_time_) : 0.01;
    dt = std::clamp(dt, 0.005, 0.05); // Giới hạn dt để tránh spike
    last_time_ = now;

    // ── Tính toán bộ điều khiển ──
    double          thrust_norm = compute_thrust_control();
    Eigen::Vector3d tau_norm    = compute_attitude_control(dt);

    // ── Gửi Thrust Setpoint ──
    px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
    thrust_msg.xyz[0] = 0.0f;
    thrust_msg.xyz[1] = 0.0f;
    thrust_msg.xyz[2] = -static_cast<float>(thrust_norm); // PX4: âm = đẩy lên (NED)
    thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    thrust_setpoint_pub_->publish(thrust_msg);

    // ── Gửi Torque Setpoint ──
    px4_msgs::msg::VehicleTorqueSetpoint torque_msg{};
    torque_msg.xyz[0] = static_cast<float>(tau_norm(0));  // Roll
    torque_msg.xyz[1] = static_cast<float>(tau_norm(1));  // Pitch
    torque_msg.xyz[2] = static_cast<float>(tau_norm(2));  // Yaw
    torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    torque_setpoint_pub_->publish(torque_msg);
}


// ============================================================
//  Tiện ích: Quaternion → Euler (ZYX convention)
// ============================================================
Eigen::Vector3d UAMAdaptiveController::quaternion_to_euler(
    const Eigen::Quaterniond& q) const
{
    Eigen::Vector3d euler;

    // Roll (x-axis)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1.0)
        euler(1) = std::copysign(M_PI / 2.0, sinp); // Gimbal lock
    else
        euler(1) = std::asin(sinp);

    // Yaw (z-axis)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

double UAMAdaptiveController::saturate(double val, double limit) const
{
    return std::clamp(val, -limit, limit);
}

void UAMAdaptiveController::publish_debug_info(const Eigen::Vector3d& tau,
                                                const Eigen::Vector3d& F_hat,
                                                double thrust)
{
    std_msgs::msg::Float64MultiArray debug_msg;
    // Layout: [pos(3), vel(3), euler(3), e1_att(3), e2_rate(3), tau(3), F_hat(3), thrust]
    debug_msg.data = {
        position_(0),     position_(1),     position_(2),
        velocity_(0),     velocity_(1),     velocity_(2),
        euler_angles_(0), euler_angles_(1), euler_angles_(2),
        e1_att_(0),       e1_att_(1),       e1_att_(2),
        e2_rate_(0),      e2_rate_(1),      e2_rate_(2),
        tau(0),           tau(1),           tau(2),
        F_hat(0),         F_hat(1),         F_hat(2),
        thrust
    };
    debug_pub_->publish(debug_msg);
}


// ============================================================
//  main()
// ============================================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UAMAdaptiveController>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
