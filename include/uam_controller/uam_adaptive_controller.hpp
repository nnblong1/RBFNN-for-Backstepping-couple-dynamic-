#pragma once

/**
 * @file uam_adaptive_controller.hpp
 *
 * Định nghĩa cấu trúc tham số và lớp điều khiển cho hệ thống UAM
 * theo đúng các công thức Backstepping Tích phân Thích nghi:
 *
 *  ┌─ Trục Z (Độ cao) ──────────────────────────────────────────┐
 *  │  e5  = z - z_des                                           │
 *  │  J5  = ∫e5 dt                                              │
 *  │  e6  = ż - ż_des + K5i·J5 + K5p·e5                        │
 *  │  U1  = m̂/(cosφ·cosθ)·(g - K5i·e5 - K5p·(...) - e5 - K5d·e6)│
 *  │  ṁ̂   = -cz·e6·(g - K5i·e5 + K5p·(...) - e5 - K5d·e6)     │
 *  └────────────────────────────────────────────────────────────┘
 *  ┌─ Roll / Pitch / Yaw ───────────────────────────────────────┐
 *  │  e7,J7,e8   →  U2 = Ixx/lx·( ... - n̂₀x/Ixx - gx/Ixx )   │
 *  │  e9,J9,e10  →  U3 = Iyy/ly·( ... - n̂₀y/Iyy - gy/Iyy )   │
 *  │  e11,J11,e12→  U4 = Izz  ·( ... - n̂₀z/Izz - gz/Izz )    │
 *  └────────────────────────────────────────────────────────────┘
 *  ┌─ Khớp tay máy 6-DoF ───────────────────────────────────────┐
 *  │  ei1 = qi - qi_des                                         │
 *  │  ei2 = q̇i - q̇i_des + Kpi·ei1                              │
 *  │  τ = (D⁻¹₇₋₁₂,₇₋₁₂)⁻¹·[Qdd - D⁻¹₇₋₁₂,₁₋₆·(τ₁₋₆-H₁₋₆)] + H₇₋₁₂ │
 *  └────────────────────────────────────────────────────────────┘
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <array>

using namespace std::chrono_literals;

// ╔══════════════════════════════════════════════════════════════╗
// ║  1.  THAM SỐ VẬT LÝ HỆ THỐNG                               ║
// ╚══════════════════════════════════════════════════════════════╝
struct UAMSystemParams {
    double mass_nominal   = 2.5;   // kg  – Giá trị m̂ khởi tạo (UAV + tay + tải ước tính)
    double gravity        = 9.81;  // m/s²
    double Ixx            = 0.029; // kg·m²  – Mô-men quán tính trục X
    double Iyy            = 0.029; // kg·m²  – Mô-men quán tính trục Y
    double Izz            = 0.055; // kg·m²  – Mô-men quán tính trục Z
    double lx             = 0.23;  // m – Tay đòn rotor trục X (cho U2)
    double ly             = 0.23;  // m – Tay đòn rotor trục Y (cho U3)
    double max_thrust     = 40.0;  // N
    double max_torque     = 5.0;   // N·m
    double max_joint_tau  = 20.0;  // N·m
};

// ╔══════════════════════════════════════════════════════════════╗
// ║  2.  HỆ SỐ BACKSTEPPING – từng trục có (Kp, Ki, Kd) riêng  ║
// ╚══════════════════════════════════════════════════════════════╝

// Trục Z  →  ký hiệu tài liệu: K5p, K5i, K5d, cz
struct AltitudeGains {
    double Kp       = 3.0;   // K5p
    double Ki       = 0.5;   // K5i
    double Kd       = 2.0;   // K5d
    double cz       = 0.8;   // hệ số thích nghi khối lượng
    double m_hat_min= 0.5;   // giới hạn dưới m̂
    double m_hat_max= 8.0;   // giới hạn trên  m̂
};

// Roll  →  K7p, K7i, K7d
struct RollGains  { double Kp=4.0, Ki=0.3, Kd=2.5; };

// Pitch →  K9p, K9i, K9d
struct PitchGains { double Kp=4.0, Ki=0.3, Kd=2.5; };

// Yaw   →  K11p, K11i, K11d
struct YawGains   { double Kp=2.0, Ki=0.1, Kd=1.5; };

struct BacksteppingGains {
    AltitudeGains alt;
    RollGains     roll;
    PitchGains    pitch;
    YawGains      yaw;
};

// Khớp tay i = 0..5  →  Kpi, Kdi
struct JointGains { double Kp=5.0, Kd=3.0; };

// ╔══════════════════════════════════════════════════════════════╗
// ║  3.  THAM SỐ RBFNN                                          ║
// ╚══════════════════════════════════════════════════════════════╝
struct RBFNNParams {
    int    num_neurons    = 25;
    int    input_dim      = 6;    // [e7,e9,e11, e8,e10,e12]
    int    output_dim     = 3;    // [n̂₀x, n̂₀y, n̂₀z]
    double learning_rate  = 0.08; // Γ
    double e_modification = 0.005;// η
    double gaussian_width = 1.5;  // b
};

// ╔══════════════════════════════════════════════════════════════╗
// ║  4.  LỚP RBFNN                                              ║
// ╚══════════════════════════════════════════════════════════════╝
class RBFNeuralNetwork {
public:
    explicit RBFNeuralNetwork(const RBFNNParams& p);

    Eigen::VectorXd compute_basis(const Eigen::VectorXd& Z) const;
    // Ước lượng: F̂ = Ŵᵀ h(Z)
    Eigen::VectorXd estimate(const Eigen::VectorXd& Z) const;
    // Cập nhật: dŴ = Γ·(h·e2ᵀ − η·‖e2‖·Ŵ)
    void update_weights(const Eigen::VectorXd& Z,
                        const Eigen::VectorXd& e2,
                        double dt);
    void reset();
    const Eigen::MatrixXd& get_weights() const { return W_hat_; }

private:
    RBFNNParams     params_;
    Eigen::MatrixXd W_hat_;
    Eigen::MatrixXd C_centers_;
    Eigen::VectorXd B_widths_;
    Eigen::MatrixXd Gamma_;
};

// ╔══════════════════════════════════════════════════════════════╗
// ║  5.  NODE ROS2                                              ║
// ╚══════════════════════════════════════════════════════════════╝
class UAMAdaptiveController : public rclcpp::Node {
public:
    explicit UAMAdaptiveController();
    ~UAMAdaptiveController() = default;

private:
    // ── Tham số ──
    UAMSystemParams   sys_;
    BacksteppingGains bs_;
    RBFNNParams       rbfnn_params_;
    std::array<JointGains, 6> jg_;   // hệ số 6 khớp

    // ── AI ──
    std::unique_ptr<RBFNeuralNetwork> rbfnn_;

    // ── Trạng thái UAV ──
    double phi_   = 0, theta_  = 0, psi_   = 0;     // Góc Euler
    double phid_  = 0, thetad_ = 0, psid_  = 0;     // Tốc độ góc
    double px_    = 0, py_     = 0, pz_    = 0;     // Vị trí
    double vx_    = 0, vy_     = 0, vz_    = 0;     // Vận tốc
    Eigen::Quaterniond q_imu_;

    // ── Trạng thái tích phân Backstepping ──
    // Trục Z
    double J5_=0, e5_=0, e6_=0;
    // Roll
    double J7_=0, e7_=0, e8_=0;
    // Pitch
    double J9_=0, e9_=0, e10_=0;
    // Yaw
    double J11_=0, e11_=0, e12_=0;

    // ── Thích nghi khối lượng ──
    double m_hat_ = 0.0;   // m̂, khởi tạo bằng mass_nominal

    // ── Nhiễu ước lượng từ cánh tay ──
    // RBFNN ước lượng n̂₀ = [n̂₀x, n̂₀y, n̂₀z]
    double n0x_hat_ = 0, n0y_hat_ = 0, n0z_hat_ = 0;
    // Mô-men trọng lực do dịch trọng tâm g_cog = [gx, gy, gz]
    double gx_cog_  = 0, gy_cog_  = 0, gz_cog_  = 0;
    // Tín hiệu bù tiến dự báo từ LSTM
    Eigen::Vector3d lstm_ff_ = Eigen::Vector3d::Zero();

    // ── Trạng thái khớp tay máy ──
    static constexpr int N_JOINTS = 6;
    std::array<double, N_JOINTS> q_{},  dq_{};
    std::array<double, N_JOINTS> qd_{}, dqd_{};
    // Ma trận động lực học tổng hệ thống (12×12)
    Eigen::MatrixXd D_inv_  = Eigen::MatrixXd::Identity(12, 12);
    Eigen::VectorXd H_vec_  = Eigen::VectorXd::Zero(12);
    bool            dyn_ready_ = false;

    // ── Điểm đặt ──
    double x_des_=0, y_des_=0, z_des_=1.5;
    double xd_des_=0, yd_des_=0, zd_des_=0;
    double psi_des_=0;

    // ── Cờ ──
    bool has_odom_   = false;
    bool has_joints_ = false;
    bool odom_initialized_ = false;
    double last_t_   = 0.0;

    // ── ROS2 I/O ──
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr    offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr  torque_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr  thrust_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr         motors_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      joint_tau_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr     status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr      lstm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr      sp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dyn_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── Callbacks ──
    void odom_cb  (const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr   msg);
    void lstm_cb  (const geometry_msgs::msg::Vector3::SharedPtr    msg);
    void sp_cb    (const geometry_msgs::msg::Vector3::SharedPtr    msg);
    void joint_cb (const sensor_msgs::msg::JointState::SharedPtr   msg);
    void dyn_cb   (const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // ── Hàm điều khiển ──
    void control_loop();
    void publish_offboard_mode();

    // Trục Z: trả về U1, cập nhật m̂ và tích phân J5
    double compute_altitude_control(double dt);

    // Tư thế: trả về [U2, U3, U4], cập nhật RBFNN và J7/J9/J11
    Eigen::Vector3d compute_attitude_control(double dt);

    // Khớp tay: trả về τ₁..τ₆ theo phương trình hệ thống đầy đủ
    Eigen::VectorXd compute_joint_control();

    // Ước lượng g_cog từ trạng thái khớp hiện tại
    void update_cog_disturbance();

    // Tiện ích
    Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q) const;
    double          sat(double v, double lim) const;
    void            declare_params();
    void            publish_debug(double U1, const Eigen::Vector3d& U234,
                                  const Eigen::VectorXd& tau_j);
};