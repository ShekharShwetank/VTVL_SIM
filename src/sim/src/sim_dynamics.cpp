#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vtvl_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

class SimDynamicsNode : public rclcpp::Node
{
public:
  SimDynamicsNode()
  : Node("sim_dynamics_node"), last_throttle_(0.0)
  {
    // Declare parameters for vehicle properties
    this->declare_parameter<double>("vehicle.dry_mass", 2000.0); // kg
    this->declare_parameter<double>("vehicle.prop_mass", 10000.0); // kg
    this->declare_parameter<double>("engine.max_thrust", 150000.0); // Newtons
    this->declare_parameter<double>("engine.isp", 300.0); // Specific Impulse in seconds
    this->declare_parameter<double>("sim.gravity", 9.81); // m/s^2
    this->declare_parameter<int>("sim.update_rate_hz", 100);

    // Get parameters
    dry_mass_ = this->get_parameter("vehicle.dry_mass").as_double();
    prop_mass_ = this->get_parameter("vehicle.prop_mass").as_double();
    max_thrust_ = this->get_parameter("engine.max_thrust").as_double();
    isp_ = this->get_parameter("engine.isp").as_double();
    gravity_ = this->get_parameter("sim.gravity").as_double();
    const int update_rate = this->get_parameter("sim.update_rate_hz").as_int();

    // Initialize state variables
    altitude_ = 0.0;
    velocity_ = 0.0;
    current_mass_ = dry_mass_ + prop_mass_;

    // Create publisher for vehicle state
    state_publisher_ = this->create_publisher<vtvl_msgs::msg::VehicleState>("/vehicle/state", 10);

    // Create subscriber for throttle command
    throttle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/actuators/throttle", 10, std::bind(&SimDynamicsNode::throttle_callback, this, std::placeholders::_1));

    // Create a timer to run the physics integration loop
    const auto update_period = std::chrono::milliseconds(1000 / update_rate);
    timer_ = this->create_wall_timer(
      update_period, std::bind(&SimDynamicsNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "1-DoF Dynamics Simulator started.");
  }

private:
  void throttle_callback(const std_msgs::msg::Float64 & msg)
  {
    // Clamp throttle value between 0.0 and 1.0
    last_throttle_ = std::max(0.0, std::min(1.0, msg.data));
  }

  void timer_callback()
  {
    // --- Physics Integration (Euler method) ---
    double dt = 1.0 / this->get_parameter("sim.update_rate_hz").as_int();
    
    // 1. Calculate thrust and mass flow rate
    double thrust = 0.0;
    double mass_dot = 0.0;
    if (current_mass_ > dry_mass_ && last_throttle_ > 0.0) {
      thrust = last_throttle_ * max_thrust_;
      mass_dot = -thrust / (isp_ * 9.81); // Tsiolkovsky rocket equation
    }

    // 2. Calculate forces and acceleration
    double force_gravity = current_mass_ * gravity_;
    double net_force = thrust - force_gravity;
    double acceleration = net_force / current_mass_;

    // 3. Integrate to find new velocity and altitude
    velocity_ += acceleration * dt;
    altitude_ += velocity_ * dt;
    current_mass_ += mass_dot * dt;
    
    // 4. Enforce ground constraint
    if (altitude_ <= 0.0) {
      altitude_ = 0.0;
      velocity_ = 0.0;
    }

    // Ensure mass doesn't go below dry mass
    if (current_mass_ < dry_mass_) {
      current_mass_ = dry_mass_;
    }

    // --- Publish the new state ---
    auto state_msg = vtvl_msgs::msg::VehicleState();
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.header.frame_id = "inertial";
    state_msg.position.z = altitude_;
    state_msg.velocity.z = velocity_;
    state_msg.mass = current_mass_;

    state_publisher_->publish(state_msg);
  }

  // ROS 2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vtvl_msgs::msg::VehicleState>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_subscriber_;

  // State variables
  double altitude_;
  double velocity_;
  double current_mass_;
  double last_throttle_;
  
  // Parameters
  double dry_mass_, prop_mass_, max_thrust_, isp_, gravity_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
