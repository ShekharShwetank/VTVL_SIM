#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vtvl_msgs/msg/vehicle_state.hpp"

using namespace std::chrono_literals;

class PidAltControllerNode : public rclcpp::Node
{
public:
  PidAltControllerNode()
  : Node("pid_alt_controller_node"), current_altitude_(0.0), integral_error_(0.0), prev_error_(0.0)
  {
    // Declare parameters for PID gains and setpoint
    this->declare_parameter<double>("pid.kp", 0.1);  // Proportional gain
    this->declare_parameter<double>("pid.ki", 0.01); // Integral gain
    this->declare_parameter<double>("pid.kd", 0.2);  // Derivative gain
    this->declare_parameter<double>("pid.setpoint_altitude", 100.0); // Target altitude in meters
    this->declare_parameter<int>("controller.update_rate_hz", 50);

    // Get parameters
    kp_ = this->get_parameter("pid.kp").as_double();
    ki_ = this->get_parameter("pid.ki").as_double();
    kd_ = this->get_parameter("pid.kd").as_double();
    setpoint_altitude_ = this->get_parameter("pid.setpoint_altitude").as_double();
    const int update_rate = this->get_parameter("controller.update_rate_hz").as_int();

    // Create publisher for throttle commands
    throttle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/actuators/throttle", 10);

    // Create subscriber for vehicle state
    state_subscriber_ = this->create_subscription<vtvl_msgs::msg::VehicleState>(
      "/vehicle/state", 10, std::bind(&PidAltControllerNode::state_callback, this, std::placeholders::_1));

    // Create a timer to run the control loop
    const auto update_period = std::chrono::milliseconds(1000 / update_rate);
    timer_ = this->create_wall_timer(
      update_period, std::bind(&PidAltControllerNode::control_loop_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "PID Altitude Controller started. Target: %.2f m", setpoint_altitude_);
  }

private:
  void state_callback(const vtvl_msgs::msg::VehicleState & msg)
  {
    current_altitude_ = msg.position.z;
  }

  void control_loop_callback()
  {
    double dt = 1.0 / this->get_parameter("controller.update_rate_hz").as_int();

    // --- PID Calculation ---
    double error = setpoint_altitude_ - current_altitude_;
    
    // Integral term (with anti-windup)
    integral_error_ += error * dt;
    integral_error_ = std::max(-2.0, std::min(2.0, integral_error_)); // Clamp integral term

    // Derivative term
    double derivative_error = (error - prev_error_) / dt;
    
    // PID output
    double output = (kp_ * error) + (ki_ * integral_error_) + (kd_ * derivative_error);
    
    // Store error for next iteration
    prev_error_ = error;

    // --- Throttle Command ---
    // A simple gravity feed-forward term can help
    // For now, we'll just map output to throttle
    double throttle_cmd = std::max(0.0, std::min(1.0, output));

    auto throttle_msg = std_msgs::msg::Float64();
    throttle_msg.data = throttle_cmd;
    throttle_publisher_->publish(throttle_msg);
  }

  // ROS 2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_publisher_;
  rclcpp::Subscription<vtvl_msgs::msg::VehicleState>::SharedPtr state_subscriber_;

  // PID variables
  double kp_, ki_, kd_;
  double setpoint_altitude_;
  double current_altitude_;
  double integral_error_;
  double prev_error_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidAltControllerNode>());
  rclcpp::shutdown();
  return 0;
}
