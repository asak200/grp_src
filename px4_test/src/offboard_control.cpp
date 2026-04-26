#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/qos.hpp"

#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std::placeholders;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		this->declare_parameter("target_system_id", 1);
		this->get_parameter("target_system_id", target_system_id);
		std::string system_id = std::to_string(target_system_id-1); 

		rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
		qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		
		// "/px4_"+system_id+      <-  copy this and paste it beside the topic names when doing multidrone simulation
		//initialize the subscribers/publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>
			("/fmu/in/offboard_control_mode", qos_profile);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>
			("/fmu/in/trajectory_setpoint", qos_profile);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>
			("/fmu/in/vehicle_command", qos_profile);
		user_command_change_control_type_listener = this->create_subscription<OffboardControlMode>
			("/control_type_"+system_id, 10, std::bind(&OffboardControl::controltype_callback, this, std::placeholders::_1));
		user_command_lintener_ = this->create_subscription<TrajectorySetpoint>
			("/set_point_"+system_id, 10, std::bind(&OffboardControl::trajectory_callback, this, std::placeholders::_1));


		offboard_setpoint_counter_ = 0;
		this->trajectory_msg.position = {0.0, -0.0, -15.0};
		this->trajectory_msg.yaw = 0; // [-PI:PI]

		this->offboard_msg.position = true;
		this->offboard_msg.velocity = false;
		this->offboard_msg.acceleration = false;
		this->offboard_msg.attitude = false;
		this->offboard_msg.body_rate = false;

		// this->arm();
		// RCLCPP_INFO(this->get_logger(), "The arm command send");
		
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
				vel_command_lintener_ = this->create_subscription<Twist>
					("cmd_vel_joy", 10, std::bind(&OffboardControl::cmd_velCallback, this, _1));
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 50) {
				offboard_setpoint_counter_++;
			}else{

			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);

		
	}

	void arm();
	void disarm();

private:

	

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<OffboardControlMode>::SharedPtr user_command_change_control_type_listener;
	rclcpp::Subscription<TrajectorySetpoint>::SharedPtr user_command_lintener_;
	rclcpp::Subscription<Twist>::SharedPtr vel_command_lintener_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	int target_system_id;
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	TrajectorySetpoint trajectory_msg{};
	OffboardControlMode offboard_msg{};

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

	void controltype_callback(const OffboardControlMode::SharedPtr msg);
	void trajectory_callback(const TrajectorySetpoint::SharedPtr msg);
	void cmd_velCallback(const Twist::SharedPtr msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	// RCLCPP_INFO(this->get_logger(), "pose: %d vel: %d", this->offboard_msg.position, this->offboard_msg.velocity);
	offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(this->offboard_msg);
}

/**
 * @brief Chanhes the command order (waypoint or velocity)
 */
void OffboardControl::controltype_callback(const OffboardControlMode::SharedPtr msg){
	this->offboard_msg.position = msg->position;
	this->offboard_msg.velocity = msg->velocity;
	this->offboard_msg.acceleration = false;
	this->offboard_msg.attitude = false;
	this->offboard_msg.body_rate = false;
	this->offboard_msg.thrust_and_torque = false;
	this->offboard_msg.direct_actuator = false;
}

/**
 * @brief Chanhes the trajectory setpoint if a new setpoint is sent
 */
void OffboardControl::trajectory_callback(const TrajectorySetpoint::SharedPtr msg){
	this->trajectory_msg.position = msg->position;
	this->trajectory_msg.velocity = msg->velocity;
	this->trajectory_msg.yaw = msg->yaw;
	this->trajectory_msg.yawspeed = msg->yawspeed;
}

/**
 * @brief Chanhes the trajectory setpoint using cmd_vel
 */
void OffboardControl::cmd_velCallback(const Twist::SharedPtr msg){
	this->offboard_msg.position = false;
	this->offboard_msg.velocity = true;
	// RCLCPP_INFO(this->get_logger(), "control type: %d %d", this->offboard_msg.position, this->offboard_msg.velocity);

	this->trajectory_msg.position = {NAN, NAN, NAN};
	this->trajectory_msg.yaw = NAN;
	this->trajectory_msg.velocity = {(float)msg->linear.x, (float)msg->linear.y, (float)msg->linear.z};
	this->trajectory_msg.yawspeed = msg->angular.z * 3.14159 / 180.;
}

/**
 * @brief Publish a trajectory setpoint 
 * 
 * (this function must work at at least 5Hz for the offboard control not to fall apart)
 */
void OffboardControl::publish_trajectory_setpoint()
{
	this->trajectory_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(this->trajectory_msg);

	std::ostringstream oss;
	oss << this->trajectory_msg.position[0]; oss << ", "; oss << this->trajectory_msg.position[1]; oss << ", "; oss << this->trajectory_msg.position[2]; oss << ", ";
	// RCLCPP_INFO(this->get_logger(), "positition: %s", oss.str().c_str());
	oss = std::ostringstream();
	oss << this->trajectory_msg.velocity[0]; oss << ", "; oss << this->trajectory_msg.velocity[1]; oss << ", "; oss << this->trajectory_msg.velocity[2]; oss << ", ";
	// RCLCPP_INFO(this->get_logger(), "vel: %s", oss.str().c_str());
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = this->target_system_id;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
