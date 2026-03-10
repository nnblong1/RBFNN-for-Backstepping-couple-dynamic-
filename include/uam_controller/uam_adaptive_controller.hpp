#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <array>

using namespace std::chrono_literals;

// ============================================================
//  Cấu trúc tham số hệ thống UAM
// ============================================================
struct UAMSystemParams {
    double mass_total       = 2.5;    // kg  - Khối lượng tổng UAV + tay + tải
    double gravity          = 9.81;   // m/s²
    double Ixx              = 0.029;  // kg·m²
    double Iyy              = 0.029;  // kg·m²
    double Izz              = 0.055;  // kg·m²
    double arm_length       = 0.23;   // m   - Khoảng cách rotor-tâm
    double thrust_coeff     = 8.54858e-6; // N/(rad/s)²
    double torque_coeff     = 0.016;  // Hệ số mô-men phản lực rotor
    double max_thrust       = 40.0;   // N   - Lực đẩy tối đa
    double max_torque       = 5.0;    // N·m - Mô-men xoắn tối đa
};

// ============================================================
//  Cấu trúc tham số RBFNN
// ============================================================
struct RBFNNParams {
    int    num_neurons      = 25;     // Số nơ-ron Gaussian trong lớp ẩn
    int    input_dim        = 6;      // Kích thước vector đầu vào Z
    int    output_dim       = 3;      // Kích thước đầu ra (roll, pitch, yaw torque)
    double learning_rate    = 0.08;   // Γ - Tốc độ học thích nghi
    double e_modification   = 0.005;  // η - Tham số chống phân kỳ trọng số
    double gaussian_width   = 1.5;    // b - Độ rộng mặc định hàm cơ sở Gaussian
};

// ============================================================
//  Cấu trúc tham số Backstepping
// ============================================================
struct BacksteppingParams {
    double K1_pos           = 3.0;    // Hệ số khuếch đại sai số vị trí (vòng ngoài)
    double K2_vel           = 2.5;    // Hệ số khuếch đại sai số vận tốc (vòng trong)
    double K1_att           = 4.0;    // Hệ số khuếch đại sai số tư thế
    double K2_rate          = 3.0;    // Hệ số khuếch đại sai số tốc độ góc
};

// ============================================================
//  Lớp tính toán RBFNN
// ============================================================
class RBFNeuralNetwork {
public:
    explicit RBFNeuralNetwork(const RBFNNParams& params);

    // Tính vector kích hoạt Gaussian h(Z)
    Eigen::VectorXd compute_basis(const Eigen::VectorXd& Z) const;

    // Ước lượng nhiễu phi tuyến F_hat = W_hat^T * h(Z)
    Eigen::VectorXd estimate(const Eigen::VectorXd& Z) const;

    // Cập nhật trọng số trực tuyến theo phương trình Lyapunov
    // dW = Γ * (h * e2^T - η * ||e2|| * W_hat)
    void update_weights(const Eigen::VectorXd& Z, 
                        const Eigen::VectorXd& e2, 
                        double dt);

    // Reset trọng số về giá trị khởi tạo
    void reset();

    const Eigen::MatrixXd& get_weights() const { return W_hat_; }

private:
    RBFNNParams params_;
    Eigen::MatrixXd W_hat_;       // Ma trận trọng số ước lượng  [N x output_dim]
    Eigen::MatrixXd C_centers_;   // Ma trận trung tâm Gaussian  [N x input_dim]
    Eigen::VectorXd B_widths_;    // Vector độ rộng Gaussian     [N]
    Eigen::MatrixXd Gamma_;       // Ma trận tốc độ học          [N x N]
};

// ============================================================
//  Node ROS2 Chính: UAM Adaptive Backstepping Controller
// ============================================================
class UAMAdaptiveController : public rclcpp::Node {
public:
    explicit UAMAdaptiveController();
    ~UAMAdaptiveController() = default;

private:
    // ----- Tham số hệ thống -----
    UAMSystemParams    sys_params_;
    RBFNNParams        rbfnn_params_;
    BacksteppingParams bs_params_;

    // ----- Đối tượng RBFNN -----
    std::unique_ptr<RBFNeuralNetwork> rbfnn_;

    // ----- Trạng thái UAV -----
    Eigen::Vector3d position_;       // [x, y, z] trong Inertial Frame
    Eigen::Vector3d velocity_;       // [vx, vy, vz]
    Eigen::Vector3d euler_angles_;   // [roll, pitch, yaw]
    Eigen::Vector3d angular_rate_;   // [p, q, r] trong Body Frame
    Eigen::Quaterniond quaternion_;

    // ----- Tín hiệu điều khiển -----
    Eigen::Vector3d lstm_ff_torque_; // Tín hiệu bù tiến từ LSTM Python node
    double          total_thrust_;   // Lực đẩy tổng [N]

    // ----- Điểm bay tham chiếu mong muốn -----
    Eigen::Vector3d position_desired_;
    Eigen::Vector3d euler_desired_;
    double          yaw_desired_;

    // ----- Biến trạng thái Backstepping -----
    Eigen::Vector3d e1_pos_;         // Sai số vị trí
    Eigen::Vector3d e2_vel_;         // Sai số vận tốc
    Eigen::Vector3d e1_att_;         // Sai số tư thế
    Eigen::Vector3d e2_rate_;        // Sai số tốc độ góc
    Eigen::Vector3d virtual_vel_;    // Tốc độ ảo α

    // ----- Cờ trạng thái -----
    bool is_offboard_mode_    = false;
    bool has_odometry_        = false;
    uint64_t offboard_counter_ = 0;
    double   last_time_       = 0.0;

    // ----- ROS2 Publishers -----
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr    offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr  torque_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr  thrust_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       debug_pub_;

    // ----- ROS2 Subscribers -----
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr     status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr      lstm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr      setpoint_sub_;

    // ----- Timer điều khiển -----
    rclcpp::TimerBase::SharedPtr control_timer_;  // 100 Hz

    // ----- Callbacks -----
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void lstm_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void setpoint_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    // ----- Hàm điều khiển chính -----
    void control_loop();
    void publish_offboard_control_mode();

    // ----- Backstepping phân cấp -----
    Eigen::Vector3d compute_attitude_control(double dt);
    double          compute_thrust_control();

    // ----- Tiện ích -----
    Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) const;
    double          saturate(double val, double limit) const;
    void            publish_debug_info(const Eigen::Vector3d& tau,
                                       const Eigen::Vector3d& F_hat,
                                       double thrust);
};
