#ifndef HW_INTERFACE_SSMR_HPP
#define HW_INTERFACE_SSMR_HPP

#include "hardware_interface/system_interface.hpp"
#include "hw_interface_ssmr/comm_class.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <memory>

namespace hw_interface_ssmr {

class SSMR_HWInterface : public hardware_interface::SystemInterface 
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    rclcpp::Logger get_logger() const { return *logger_; }

private:
    std::shared_ptr<SerialCommDriver> driver_;

    double left_wheel_velocity_ = 0.0;
    double left_wheel_position_ = 0.0;
    double right_wheel_velocity_ = 0.0;
    double right_wheel_position_ = 0.0;

    double left_cmd_pwm_ = 0.0;
    double right_cmd_pwm_ = 0.0;

    std::string port_;
    std::shared_ptr<rclcpp::Logger> logger_;
};

} // namespace hw_interface_ssmr

#endif // HW_INTERFACE_SSMR_HPP