#ifndef LASER_USV_PX4_API__API_NODE_HPP_
#define LASER_USV_PX4_API__API_NODE_HPP_

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace laser_usv_px4_api
{
class ApiNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ApiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ApiNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

	// IMU 
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_sensor_combined_;
  void subSensorCombined(const px4_msgs::msg::SensorCombined::SharedPtr msg);

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

	// odometry	 
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
  void subOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

	// vehicle_command
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_vehicle_command_;
 	void pubVehicleCommand(int command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

	// vehicle_control_mode
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr sub_vehicle_control_mode_;
  void subVehicleControlMode(const px4_msgs::msg::VehicleControlMode &msg);

	// offboard_control_mode
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_control_mode_;

	double _rate_pub_offboard_control_mode_;
	rclcpp::TimerBase::SharedPtr tmr_pub_offboard_control_mode_;
	void tmrPubOffboardControlMode();
	
	// srv arm
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_arm_;
  void srvArm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

	// srv disarm
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disarm_;
  void srvDisarm(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

	// rover actuator motors from actuators topic
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_actuators_;
  void subActuators(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::ActuatorMotors>::SharedPtr pub_actuator_motors_;

	double _rate_pub_actuator_motors_;
	rclcpp::TimerBase::SharedPtr tmr_pub_actuator_motors_;
	void tmrPubActuatorMotors();

	px4_msgs::msg::ActuatorMotors actuator_motors_msg_;

  bool _offboard_position_{false};
  bool _offboard_velocity_{false};
  bool _offboard_acceleration_{false};
  bool _offboard_attitude_{false};
  bool _offboard_thrust_and_torque_{false};
  bool _offboard_body_rate_{false};
  bool _offboard_direct_actuator_{false};

  bool is_active_{false};
	bool offboard_is_enabled_{false};

  Eigen::Quaterniond ned_enu_quaternion_rotation_;
  Eigen::Quaterniond frd_flu_rotation_;
  Eigen::Affine3d frd_flu_affine_;
  Eigen::PermutationMatrix<3> ned_enu_reflection_xy_;
  Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z_;

  Eigen::Vector3d enuToNed(Eigen::Vector3d p);
  Eigen::Vector3d frdToFlu(Eigen::Vector3d p);
  Eigen::Quaterniond enuToNedOrientation(Eigen::Quaterniond q);
};
} // namespace laser_usv_px4_api

#endif
