#include "laser_usv_px4_api/api_node.hpp"

namespace laser_usv_px4_api
{
// ApiNode() <<< 
ApiNode::ApiNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("api_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

	declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));
	declare_parameter("rate.pub_actuator_motors", rclcpp::ParameterValue(500.0));

  ned_enu_quaternion_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
    																								Eigen::AngleAxisd(0,      Eigen::Vector3d::UnitY()) *
    																								Eigen::AngleAxisd(M_PI,   Eigen::Vector3d::UnitX()));

  frd_flu_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(0,     Eigen::Vector3d::UnitZ()) *
      																	 Eigen::AngleAxisd(0,     Eigen::Vector3d::UnitY()) *
      																	 Eigen::AngleAxisd(M_PI,  Eigen::Vector3d::UnitX()));

  frd_flu_affine_ = Eigen::Affine3d(frd_flu_rotation_);

  ned_enu_reflection_xy_ = Eigen::PermutationMatrix<3>(Eigen::Vector3i(1, 0, 2));
  ned_enu_reflection_z_  = Eigen::DiagonalMatrix<double, 3>(1, 1, -1);
}
// >>>

// ~ApiNode() <<< 
ApiNode::~ApiNode() {
}
// >>>

// on_configure() <<< 
CallbackReturn ApiNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}
// >>>

// on_activate() <<< 
CallbackReturn ApiNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_imu_->on_activate();
  pub_odometry_->on_activate();
	pub_vehicle_command_->on_activate();
	pub_offboard_control_mode_->on_activate();
	// pub_rover_speed_setpoint_->on_activate();
	// pub_rover_rate_setpoint_->on_activate();
	// pub_rover_attitude_setpoint_->on_activate();
	pub_actuator_motors_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}
// >>> 

// on_deactivate() <<< 
CallbackReturn ApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_imu_->on_deactivate();
  pub_odometry_->on_deactivate();
	pub_vehicle_command_->on_deactivate();
	pub_offboard_control_mode_->on_deactivate();
	// pub_rover_speed_setpoint_->on_deactivate();
	// pub_rover_rate_setpoint_->on_deactivate();
	// pub_rover_attitude_setpoint_->on_deactivate();
	pub_actuator_motors_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}
// >>> 

// on_cleanup() <<<
CallbackReturn ApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pub_imu_.reset();
  pub_odometry_.reset();
	pub_vehicle_command_.reset();
	pub_offboard_control_mode_.reset();
	// pub_rover_speed_setpoint_.reset();
	// pub_rover_rate_setpoint_.reset();
	// pub_rover_attitude_setpoint_.reset();
	pub_actuator_motors_.reset();

  sub_sensor_combined_.reset();
  sub_odometry_.reset();
  sub_vehicle_control_mode_.reset();
  // sub_cmd_vel_.reset();

	tmr_pub_offboard_control_mode_->reset();

  return CallbackReturn::SUCCESS;
}
// >>>

// on_shutdown() <<<
CallbackReturn ApiNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}
// >>>

// getParameters() <<<
void ApiNode::getParameters() {

	get_parameter("rate.pub_offboard_control_mode", _rate_pub_offboard_control_mode_);
	get_parameter("rate.pub_actuator_motors", _rate_pub_actuator_motors_);
}
// >>>

// configPubSub() <<<
void ApiNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");


  sub_sensor_combined_ = this->create_subscription<px4_msgs::msg::SensorCombined>("fmu/out/sensor_combined", rclcpp::SensorDataQoS(),
																																									std::bind(&ApiNode::subSensorCombined, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
																																						std::bind(&ApiNode::subOdometry, this, std::placeholders::_1));
  sub_vehicle_control_mode_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>("fmu/out/vehicle_control_mode", rclcpp::SensorDataQoS(),
                                                                                 					 std::bind(&ApiNode::subVehicleControlMode, this, std::placeholders::_1));
  // sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("rahcm/cmd_vel", rclcpp::SensorDataQoS(), 
																																			// std::bind(&ApiNode::subCmdVel, this, std::placeholders::_1));

  sub_actuators_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("rahcm/actuators", rclcpp::SensorDataQoS(), 
																																			std::bind(&ApiNode::subActuators, this, std::placeholders::_1));

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
	pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
	pub_offboard_control_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
	// pub_rover_speed_setpoint_ = this->create_publisher<px4_msgs::msg::RoverSpeedSetpoint>("/fmu/in/rover_speed_setpoint", 10);
	// pub_rover_rate_setpoint_ = this->create_publisher<px4_msgs::msg::RoverRateSetpoint>("/fmu/in/rover_rate_setpoint", 10);
	// pub_rover_attitude_setpoint_ = this->create_publisher<px4_msgs::msg::RoverAttitudeSetpoint>("/fmu/in/rover_attitude_setpoint", 10);
	pub_actuator_motors_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
}
// >>>

// configTimers() <<<
void ApiNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

	tmr_pub_offboard_control_mode_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_offboard_control_mode_),
                                                         std::bind(&ApiNode::tmrPubOffboardControlMode, this), nullptr);
	tmr_pub_actuator_motors_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_actuator_motors_),
                                                         std::bind(&ApiNode::tmrPubActuatorMotors, this), nullptr);
}
// >>>

// configServices() <<<
void ApiNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

	srv_arm_    = create_service<std_srvs::srv::Trigger>("arm", std::bind(&ApiNode::srvArm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>("disarm", std::bind(&ApiNode::srvDisarm, this, std::placeholders::_1, std::placeholders::_2));
}
// >>>

// TODO: velocity + rate offboard control mode
// subCmdVel{} <<< 
// void ApiNode::subCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
// {
//  	if (!is_active_) {
//     return;
//   }
//
//  	if (!offboard_is_enabled_) {
//     return;
//   }
	
	// px4_msgs::msg::RoverSpeedSetpoint r_speed_msg{};
	// px4_msgs::msg::RoverRateSetpoint r_rate_msg{};
	// px4_msgs::msg::RoverAttitudeSetpoint r_attitude_msg{};
	//
	// int64_t timestamp = get_clock()->now().nanoseconds() / 1000;
	//
	// r_speed_msg.timestamp = timestamp;
	// r_rate_msg.timestamp = timestamp;
	// r_attitude_msg.timestamp = timestamp;
	//
	// // speed setpoint
	// Eigen::Vector3d lin_vel(msg->linear.x,
	// 											 	msg->linear.y, 
	// 											  msg->linear.z);
	// lin_vel = enuToNed(lin_vel);
	//
	// r_speed_msg.speed_body_x = lin_vel(1); 
	// r_speed_msg.speed_body_y = lin_vel(0); 
	//
	// // rate setpoint
	// Eigen::Vector3d ang_vel(msg->angular.x,
	// 											 	msg->angular.y, 
	// 											  msg->angular.z);
	// ang_vel = enuToNed(ang_vel);
	//
	// r_rate_msg.yaw_rate_setpoint = ang_vel(2); 
	//
	// // attitude setpoint
	// r_attitude_msg.yaw_setpoint = NAN; 

	// pub_rover_speed_setpoint_->publish(r_speed_msg);
	// pub_rover_rate_setpoint_->publish(r_rate_msg);
	// pub_rover_attitude_setpoint_->publish(r_attitude_msg);
// }
// >>>

// subActuators() <<<
void ApiNode::subActuators(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
{
 	if (!is_active_) {
    return;
  }

	if (!offboard_is_enabled_) {
    return;
  }

	actuator_motors_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
	actuator_motors_msg_.timestamp = 0;

	actuator_motors_msg_.reversible_flags = 3; // all motors are reversible
	
	for (size_t i = 0; i < msg->data.size(); i++) {
		actuator_motors_msg_.control[i] = msg->data[i];
	}
}
// >>>

// subVehicleControlMode{} <<<
void ApiNode::subVehicleControlMode(const px4_msgs::msg::VehicleControlMode &msg) 
{
  if (!is_active_) {
    return;
  }

  offboard_is_enabled_ = msg.flag_control_offboard_enabled;
}
// >>>

// subSensorCombined() <<< 
void ApiNode::subSensorCombined(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{
 if (!is_active_) {
    return;
  }

  sensor_msgs::msg::Imu imu_msg{};
  imu_msg.header.stamp = get_clock()->now();
  imu_msg.header.frame_id = "imu_link";

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2];
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_msg.angular_velocity.x = frd_to_flu(0);
  imu_msg.angular_velocity.y = frd_to_flu(1);
  imu_msg.angular_velocity.z = frd_to_flu(2);

  frd_to_flu << msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2];
  frd_to_flu = frdToFlu(frd_to_flu);

  imu_msg.linear_acceleration.x = frd_to_flu(0);
  imu_msg.linear_acceleration.y = frd_to_flu(1);
  imu_msg.linear_acceleration.z = frd_to_flu(2);

  pub_imu_->publish(imu_msg);
}
/// >>>

// subOdometry() <<<
void ApiNode::subOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
 if (!is_active_) {
    return;
  }

	nav_msgs::msg::Odometry odom{};
  odom.header.stamp = get_clock()->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  Eigen::Vector3d ned_to_enu_tf(msg->position[0],
																msg->position[1],
																msg->position[2]);
  ned_to_enu_tf = enuToNed(ned_to_enu_tf);

  odom.pose.pose.position.x = ned_to_enu_tf(0);
  odom.pose.pose.position.y = ned_to_enu_tf(1);
  odom.pose.pose.position.z = ned_to_enu_tf(2);

  Eigen::Quaterniond ned_to_enu_orientation_tf(msg->q[0],
																							 msg->q[1],
																							 msg->q[2],
																							 msg->q[3]);
  ned_to_enu_orientation_tf = enuToNedOrientation(ned_to_enu_orientation_tf);
  ned_to_enu_orientation_tf = ned_to_enu_orientation_tf.normalized();
  ned_to_enu_orientation_tf.coeffs() *= -1;

  odom.pose.pose.orientation.x = ned_to_enu_orientation_tf.x();
  odom.pose.pose.orientation.y = ned_to_enu_orientation_tf.y();
  odom.pose.pose.orientation.z = ned_to_enu_orientation_tf.z();
  odom.pose.pose.orientation.w = ned_to_enu_orientation_tf.w();

  odom.pose.covariance = {msg->position_variance[0],    0, 0, 0, 0, 0, 0, msg->position_variance[1],    0, 0, 0, 0, 0, 0,
                          msg->position_variance[2],    0, 0, 0, 0, 0, 0, msg->orientation_variance[0], 0, 0, 0, 0, 0, 0,
                          msg->orientation_variance[1], 0, 0, 0, 0, 0, 0, msg->orientation_variance[2]};

  ned_to_enu_tf(0) = msg->velocity[0];
  ned_to_enu_tf(1) = msg->velocity[1];
  ned_to_enu_tf(2) = msg->velocity[2];
  ned_to_enu_tf    = enuToNed(ned_to_enu_tf);
  ned_to_enu_tf    = ned_to_enu_orientation_tf.conjugate().normalized().toRotationMatrix() * ned_to_enu_tf;

  odom.twist.twist.linear.x = ned_to_enu_tf(0);
  odom.twist.twist.linear.y = ned_to_enu_tf(1);
  odom.twist.twist.linear.z = ned_to_enu_tf(2);

  Eigen::Vector3d frd_to_flu;
  frd_to_flu << msg->angular_velocity[0], 
								msg->angular_velocity[1], 
								msg->angular_velocity[2];
  frd_to_flu = frdToFlu(frd_to_flu);

  odom.twist.twist.angular.x = frd_to_flu(0);
  odom.twist.twist.angular.y = frd_to_flu(1);
  odom.twist.twist.angular.z = frd_to_flu(2);

  odom.twist.covariance = {msg->velocity_variance[0], 0, 0, 0, 0, 0, 0, msg->velocity_variance[1], 0, 0, 0, 0, 0, 0,
                           msg->velocity_variance[2], 0, 0, 0, 0, 0, 0, msg->velocity_variance[0], 0, 0, 0, 0, 0, 0,
                           msg->velocity_variance[1], 0, 0, 0, 0, 0, 0, msg->velocity_variance[2]};

  pub_odometry_->publish(odom);
}

// >>>

// tmrPubOffboardControlMode() <<<
void ApiNode::tmrPubOffboardControlMode() {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode msg{};

  msg.position          = false;
  msg.velocity          = false;
  msg.acceleration      = false;
  msg.attitude          = false;
  msg.thrust_and_torque = false;
  msg.body_rate       	= false;
  msg.direct_actuator 	= true;
 
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;

  pub_offboard_control_mode_->publish(msg);
}
// >>>

// tmrPubActuatorMotors() <<<
void ApiNode::tmrPubActuatorMotors() {
  if (!is_active_) {
    return;
  }

	if (!offboard_is_enabled_) {
    return;
  }

	pub_actuator_motors_->publish(actuator_motors_msg_);
}
// >>>

// pubVehicleCommand() <<<
void ApiNode::pubVehicleCommand(int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::VehicleCommand msg{};
  msg.param1           = param1;
  msg.param2           = param2;
  msg.param3           = param3;
  msg.param4           = param4;
  msg.param5           = param5;
  msg.param6           = param6;
  msg.param7           = param7;
  msg.command          = command;
  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
  pub_vehicle_command_->publish(msg);
}
// >>>

// srvArm() <<<
void ApiNode::srvArm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  pubVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  pubVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  response->success = true;
  response->message = "arm requested success";
}
// >>>

// srvDisarm() <<<
void ApiNode::srvDisarm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  pubVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  response->success = true;
  response->message = "disarm requested success";
}
// >>>

// enuToNed() <<<
Eigen::Vector3d ApiNode::enuToNed(Eigen::Vector3d p) {
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}
// >>>

// frdToFlu() <<<
Eigen::Vector3d ApiNode::frdToFlu(Eigen::Vector3d p) {
  return frd_flu_affine_ * p;
}
// >>>

// enuToOrientation() <<<
Eigen::Quaterniond ApiNode::enuToNedOrientation(Eigen::Quaterniond q) {
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}
// >>>
} // namespace laser_usv_px4_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_usv_px4_api::ApiNode)
