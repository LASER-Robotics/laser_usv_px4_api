#include "laser_usv_px4_api/api_node.hpp"

namespace laser_usv_px4_api
{
ApiNode::ApiNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("api_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

	declare_parameter("rate.pub_offboard_control_mode", rclcpp::ParameterValue(100.0));

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

ApiNode::~ApiNode() {
}

CallbackReturn ApiNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_imu_->on_activate();
  pub_odometry_->on_activate();
	pub_vehicle_command_->on_activate();
	pub_offboard_control_mode_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_imu_->on_deactivate();
  pub_odometry_->on_deactivate();
	pub_vehicle_command_->on_deactivate();
	pub_offboard_control_mode_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pub_imu_.reset();
  pub_odometry_.reset();
	pub_vehicle_command_.reset();
	pub_offboard_control_mode_.reset();

  sub_sensor_combined_.reset();
  sub_odometry_.reset();
  sub_vehicle_control_mode_.reset();

	tmr_pub_offboard_control_mode_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ApiNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}

void ApiNode::getParameters() {

	get_parameter("rate.pub_offboard_control_mode", _rate_pub_offboard_control_mode_);
}

void ApiNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");


  sub_sensor_combined_ = this->create_subscription<px4_msgs::msg::SensorCombined>("fmu/out/sensor_combined", rclcpp::SensorDataQoS(),
																																									std::bind(&ApiNode::subSensorCombined, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
																																						std::bind(&ApiNode::subOdometry, this, std::placeholders::_1));
  sub_vehicle_control_mode_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>("fmu/out/vehicle_control_mode", rclcpp::SensorDataQoS(),
                                                                                 std::bind(&ApiNode::subVehicleControlMode, this, std::placeholders::_1));

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
	pub_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
	pub_offboard_control_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
}

void ApiNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

	tmr_pub_offboard_control_mode_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_pub_offboard_control_mode_),
                                                         std::bind(&ApiNode::tmrPubOffboardControlMode, this), nullptr);
}

void ApiNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

	srv_arm_    = create_service<std_srvs::srv::Trigger>("arm", std::bind(&ApiNode::srvArm, this, std::placeholders::_1, std::placeholders::_2));
  srv_disarm_ = create_service<std_srvs::srv::Trigger>("disarm", std::bind(&ApiNode::srvDisarm, this, std::placeholders::_1, std::placeholders::_2));
}

void ApiNode::subVehicleControlMode(const px4_msgs::msg::VehicleControlMode &msg) 
{
  if (!is_active_) {
    return;
  }

  offboard_is_enabled_ = msg.flag_control_offboard_enabled;
}

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

void ApiNode::tmrPubOffboardControlMode() {
  if (!is_active_) {
    return;
  }

  px4_msgs::msg::OffboardControlMode msg{};

  msg.position          = true;
  msg.velocity          = false;
  msg.acceleration      = false;
  msg.attitude          = false;
  msg.thrust_and_torque = false;
  msg.body_rate       	= false;
  msg.direct_actuator 	= false;
 
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;

  pub_offboard_control_mode_->publish(msg);
}

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

void ApiNode::srvDisarm([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_active_) {
    return;
  }

  pubVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  response->success = true;
  response->message = "disarm requested success";
}

Eigen::Vector3d ApiNode::enuToNed(Eigen::Vector3d p) {
  return ned_enu_reflection_xy_ * (ned_enu_reflection_z_ * p);
}

Eigen::Vector3d ApiNode::frdToFlu(Eigen::Vector3d p) {
  return frd_flu_affine_ * p;
}

Eigen::Quaterniond ApiNode::enuToNedOrientation(Eigen::Quaterniond q) {
  return (ned_enu_quaternion_rotation_ * q) * frd_flu_rotation_;
}
} // namespace laser_usv_px4_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(laser_usv_px4_api::ApiNode)
