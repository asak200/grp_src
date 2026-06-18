#include "hw_interface_ssmr/mobile_based_hw_interface.hpp"
#include <sstream>

namespace hw_interface_ssmr {

hardware_interface::CallbackReturn SSMR_HWInterface::on_init(const hardware_interface::HardwareInfo & info) 
{
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("SSMR_HWInterface"));

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;
    port_ = info_.hardware_parameters["port_name"];

    if (info_.joints.size() < 2) {
        RCLCPP_FATAL(get_logger(), "Expected at least 2 joints in URDF, got %zu", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "\n\n\nI'M IN init.\n\n");

    driver_ = std::make_shared<SerialCommDriver>(port_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SSMR_HWInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "velocity", &right_wheel_velocity_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, "position", &right_wheel_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, "velocity", &left_wheel_velocity_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, "position", &left_wheel_position_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SSMR_HWInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, "pwm", &right_cmd_pwm_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, "pwm", &left_cmd_pwm_));
    return command_interfaces;
}

hardware_interface::CallbackReturn SSMR_HWInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    try {
        driver_->init();
    } catch (...) {
        RCLCPP_FATAL(get_logger(), "Failed to connect to serial port.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "\n\n\nI'M IN configure.\n\n");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SSMR_HWInterface::on_activate(const rclcpp_lifecycle::State& previous_state) 
{
    (void)previous_state;
    left_wheel_velocity_ = 0.0;
    left_wheel_position_ = 0.0;
    right_wheel_velocity_ = 0.0;
    right_wheel_position_ = 0.0;
    left_cmd_pwm_ = 0.0;
    right_cmd_pwm_ = 0.0;
    RCLCPP_INFO(get_logger(), "\n\n\nI'M IN activate.\n\n");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SSMR_HWInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state) 
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SSMR_HWInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) 
{
    (void)time; (void)period;

    std::string data = driver_->read("r");
    if (data == "Timeout" || data.empty()) {
        RCLCPP_WARN(get_logger(), "Serial read timeout or empty data.");
        return hardware_interface::return_type::OK; // Returning OK prevents crash, but logs warning
    }

    double read_left_velocity = 0.0, read_right_velocity = 0.0;
    double read_left_pose = 0.0, read_right_pose = 0.0;

    std::stringstream ss(data);
    // RCLCPP_INFO(get_logger(), "\n\n\nI'M IN read.\n\n");
    // RCLCPP_INFO(get_logger(), data.c_str());
    ss >> read_left_velocity >> read_left_pose >> read_right_velocity >> read_right_pose;

    left_wheel_velocity_ = read_left_velocity;
    right_wheel_velocity_ = read_right_velocity;
    left_wheel_position_ = read_left_pose;
    right_wheel_position_ = read_right_pose;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SSMR_HWInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period) 
{
    (void)time; (void)period;
    std::stringstream ss;
    ss << "pw: " << static_cast<int>(left_cmd_pwm_) << " " << static_cast<int>(right_cmd_pwm_);
    driver_->write(ss.str());
    return hardware_interface::return_type::OK;
}

} // namespace hw_interface_ssmr

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hw_interface_ssmr::SSMR_HWInterface, hardware_interface::SystemInterface)