#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "vtvl_msgs/msg/actuator_controls.hpp"
#include "vtvl_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

// Helper to convert quaternion to pitch angle (Euler ZYX)
double get_pitch_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
    // siny_cosp = 2 * (q.w * q.y - q.z * q.x)
    // cosy_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    // pitch = std::atan2(siny_cosp, cosy_cosp)
    // Since we are in 2D, we can simplify. Our quaternion is based only on pitch.
    // q.y = sin(pitch / 2), q.w = cos(pitch / 2)
    return 2 * atan2(q.y, q.w);
}


class PlanarControllerNode : public rclcpp::Node
{
public:
  PlanarControllerNode()
  : Node("planar_controller_node")
  {
    // Declare parameters for two PID controllers
    this->declare_parameter<double>("alt.kp", 0.1);
    this->declare_parameter<double>("alt.ki", 0.01);
    this->declare_parameter<double>("alt.kd", 0.2);
    this->declare_parameter<double>("att.kp", 2.0); // Attitude gains will be different
    this->declare_parameter<double>("att.ki", 0.5);
    this->declare_parameter<double>("att.kd", 1.0);
    this->declare_parameter<double>("setpoint.altitude", 100.0);
    this->declare_parameter<double>("setpoint.pitch", 0.0); // Target pitch in radians (0 = upright)
    this->declare_parameter<int>("controller.update_rate_hz", 50);

    // Get parameters
    alt_kp_ = this->get_parameter("alt.kp").as_double();
    alt_ki_ = this->get_parameter("alt.ki").as_double();
    alt_kd_ = this->get_parameter("alt.kd").as_double();
    att_kp_ = this->get_parameter("att.kp").as_double();
    att_ki_ = this->get_parameter("att.ki").as_double();
    att_kd_ = this->get_parameter("att.kd").as_double();
    setpoint_altitude_ = this->get_parameter("setpoint.altitude").as_double();
    setpoint_pitch_ = this->get_parameter("setpoint.pitch").as_double();
    const int update_rate = this->get_parameter("controller.update_rate_hz").as_int();

    // Create publisher and subscriber
    actuator_publisher_ = this->create_publisher<vtvl_msgs::msg::ActuatorControls>("/actuators/controls", 10);
    state_subscriber_ = this->create_subscription<vtvl_msgs::msg::VehicleState>(
      "/vehicle/state", 10, std::bind(&PlanarControllerNode::state_callback, this, std::placeholders::_1));

    // Create timer for the control loop
    const auto update_period = std::chrono::milliseconds(1000 / update_rate);
    timer_ = this->create_wall_timer(update_period, std::bind(&PlanarControllerNode::control_loop_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Planar Controller started. Target: %.2f m, %.2f rad", setpoint_altitude_, setpoint_pitch_);
  }

private:
  void state_callback(const vtvl_msgs::msg::VehicleState & msg)
  {
    current_altitude_ = msg.position.z;
    current_pitch_ = get_pitch_from_quaternion(msg.attitude);
    // current_pitch_rate_ = msg.angular_velocity.z; // For 2D, pitch rate is around Z
    current_pitch_rate_ = msg.angular_velocity.y;
  }

  void control_loop_callback()
  {
    double dt = 1.0 / this->get_parameter("controller.update_rate_hz").as_int();

    // --- Altitude PID Calculation (for Throttle) ---
    double alt_error = setpoint_altitude_ - current_altitude_;
    alt_integral_error_ += alt_error * dt;
    alt_integral_error_ = std::max(-5.0, std::min(5.0, alt_integral_error_)); // Anti-windup
    double alt_derivative_error = (alt_error - alt_prev_error_) / dt;
    double throttle_output = (alt_kp_ * alt_error) + (alt_ki_ * alt_integral_error_) + (alt_kd_ * alt_derivative_error);
    alt_prev_error_ = alt_error;

    // --- Attitude PID Calculation (for Gimbal) ---
    // This is a PD controller on attitude, using pitch rate for the D-term
    double att_error = setpoint_pitch_ - current_pitch_;
    double gimbal_output = (att_kp_ * att_error) - (att_kd_ * current_pitch_rate_);

    // --- Publish Actuator Commands ---
    auto actuator_msg = vtvl_msgs::msg::ActuatorControls();
    actuator_msg.throttle = std::max(0.0, std::min(1.0, throttle_output)); // Clamp throttle
    actuator_msg.gimbal_z = std::max(-0.2, std::min(0.2, gimbal_output)); // Clamp gimbal angle to ~11.5 deg
    
    actuator_publisher_->publish(actuator_msg);
  }

  // ROS 2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vtvl_msgs::msg::ActuatorControls>::SharedPtr actuator_publisher_;
  rclcpp::Subscription<vtvl_msgs::msg::VehicleState>::SharedPtr state_subscriber_;

  // Altitude PID variables
  double alt_kp_, alt_ki_, alt_kd_;
  double setpoint_altitude_;
  double current_altitude_ = 0.0;
  double alt_integral_error_ = 0.0;
  double alt_prev_error_ = 0.0;
  
  // Attitude PID variables
  double att_kp_, att_ki_, att_kd_;
  double setpoint_pitch_;
  double current_pitch_ = 0.0;
  double current_pitch_rate_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarControllerNode>());
  rclcpp::shutdown();
  return 0;
}