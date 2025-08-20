#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "vtvl_msgs/msg/actuator_controls.hpp"
#include "vtvl_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

// Helper function to convert pitch angle to a quaternion
geometry_msgs::msg::Quaternion to_quaternion(double pitch) {
    geometry_msgs::msg::Quaternion q;
    q.w = cos(pitch * 0.5);
    q.x = 0.0;
    q.y = sin(pitch * 0.5); // Pitch rotation is around the Y-axis
    q.z = 0.0;
    return q;
}

class SimDynamicsNode : public rclcpp::Node
{
public:
  SimDynamicsNode()
  : Node("sim_dynamics_node")
  {
    // Declare parameters
    this->declare_parameter<double>("vehicle.dry_mass", 2000.0);
    this->declare_parameter<double>("vehicle.prop_mass", 10000.0);
    this->declare_parameter<double>("vehicle.inertia_y", 50000.0); // CORRECTED: Inertia around Y for pitch
    this->declare_parameter<double>("vehicle.cg_to_engine_dist", 5.0); // meters
    this->declare_parameter<double>("engine.max_thrust", 150000.0);
    this->declare_parameter<double>("engine.isp", 300.0);
    this->declare_parameter<double>("sim.gravity", 9.81);
    this->declare_parameter<int>("sim.update_rate_hz", 100);

    // Get parameters
    dry_mass_ = this->get_parameter("vehicle.dry_mass").as_double();
    prop_mass_ = this->get_parameter("vehicle.prop_mass").as_double();
    inertia_y_ = this->get_parameter("vehicle.inertia_y").as_double(); // CORRECTED
    cg_to_engine_dist_ = this->get_parameter("vehicle.cg_to_engine_dist").as_double();
    max_thrust_ = this->get_parameter("engine.max_thrust").as_double();
    isp_ = this->get_parameter("engine.isp").as_double();
    gravity_ = this->get_parameter("sim.gravity").as_double();
    const int update_rate = this->get_parameter("sim.update_rate_hz").as_int();

    // Initialize state variables
    pos_ = {0.0, 0.0}; // [x, z]
    vel_ = {0.0, 0.0}; // [vx, vz]
    theta_ = 0.0;     // pitch angle, radians
    theta_dot_ = 0.0; // pitch rate, rad/s
    current_mass_ = dry_mass_ + prop_mass_;

    // Initialize actuator commands
    last_throttle_ = 0.0;
    last_gimbal_cmd_ = 0.0; // Using a generic name now

    // Create publisher and subscriber
    state_publisher_ = this->create_publisher<vtvl_msgs::msg::VehicleState>("/vehicle/state", 10);
    actuator_subscriber_ = this->create_subscription<vtvl_msgs::msg::ActuatorControls>(
      "/actuators/controls", 10, std::bind(&SimDynamicsNode::actuator_callback, this, std::placeholders::_1));

    // Create timer for physics loop
    const auto update_period = std::chrono::milliseconds(1000 / update_rate);
    timer_ = this->create_wall_timer(update_period, std::bind(&SimDynamicsNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "3-DoF Planar Dynamics Simulator started.");
  }

private:
  void actuator_callback(const vtvl_msgs::msg::ActuatorControls & msg)
  {
    last_throttle_ = std::max(0.0, std::min(1.0, msg.throttle));
    last_gimbal_cmd_ = msg.gimbal_z; // msg field is gimbal_z, but we use it for Y-axis pitch control
  }

  void timer_callback()
  {
    double dt = 1.0 / this->get_parameter("sim.update_rate_hz").as_int();
    
    // 1. Calculate thrust and mass flow
    double thrust_mag = (current_mass_ > dry_mass_ && last_throttle_ > 0.0) ? last_throttle_ * max_thrust_ : 0.0;
    double mass_dot = (thrust_mag > 0.0) ? -thrust_mag / (isp_ * 9.81) : 0.0;
    current_mass_ += mass_dot * dt;
    current_mass_ = std::max(current_mass_, dry_mass_);

    // 2. Calculate forces and torques
    // Body frame: Z is up, X is forward. Pitch is rotation around Y.
    double thrust_x_body = thrust_mag * sin(last_gimbal_cmd_);
    double thrust_z_body = thrust_mag * cos(last_gimbal_cmd_);
    
    // Rotate thrust into inertial frame
    double thrust_x_inertial = thrust_x_body * cos(theta_) + thrust_z_body * sin(theta_);
    double thrust_z_inertial = -thrust_x_body * sin(theta_) + thrust_z_body * cos(theta_);

    // Net forces in inertial frame
    double net_force_x = thrust_x_inertial;
    double net_force_z = thrust_z_inertial - current_mass_ * gravity_;
    
    // CORRECTED: Torque around Y-axis from thrust in X-direction
    double net_torque_y = thrust_x_body * cg_to_engine_dist_;

    // 3. Calculate accelerations
    double accel_x = net_force_x / current_mass_;
    double accel_z = net_force_z / current_mass_;
    double alpha_y = net_torque_y / inertia_y_; // CORRECTED

    // 4. Integrate to find new state
    vel_[0] += accel_x * dt;
    vel_[1] += accel_z * dt;
    pos_[0] += vel_[0] * dt;
    pos_[1] += vel_[1] * dt;
    theta_dot_ += alpha_y * dt; // CORRECTED
    theta_ += theta_dot_ * dt;
    
    // 5. Enforce ground constraint
    if (pos_[1] <= 0.0) {
      pos_[1] = 0.0;
      vel_ = {0.0, 0.0};
      theta_dot_ = 0.0;
    }

    // --- Publish the new state ---
    auto state_msg = vtvl_msgs::msg::VehicleState();
    state_msg.header.stamp = this->get_clock()->now();
    state_msg.header.frame_id = "inertial";
    state_msg.position.x = pos_[0];
    state_msg.position.z = pos_[1];
    state_msg.velocity.x = vel_[0];
    state_msg.velocity.z = vel_[1];
    state_msg.attitude = to_quaternion(theta_);
    state_msg.angular_velocity.y = theta_dot_;
    state_msg.mass = current_mass_;

    state_publisher_->publish(state_msg);
  }

  // ROS 2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vtvl_msgs::msg::VehicleState>::SharedPtr state_publisher_;
  rclcpp::Subscription<vtvl_msgs::msg::ActuatorControls>::SharedPtr actuator_subscriber_;

  // State variables
  std::vector<double> pos_;
  std::vector<double> vel_;
  double theta_;
  double theta_dot_;
  double current_mass_;
  double last_throttle_;
  double last_gimbal_cmd_;
  
  // Parameters
  double dry_mass_, prop_mass_, inertia_y_, cg_to_engine_dist_, max_thrust_, isp_, gravity_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}