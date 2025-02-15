#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h> // Use a serial library like libserial
#include <vector>
#include <string>

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ArduinoHardwareInterface() = default;

  // Step 1: Declare the command and state interfaces
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info)
  {
    // Initialize serial port
    serial_port_.Open("/dev/arduino"); // Replace with your serial port
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    // Declare command interfaces (velocity commands for each wheel)
    for (const auto & joint : info.joints) {
      for (const auto & command : joint.command_interfaces) {
        if (command.name == "velocity") {
          command_interfaces_.push_back(joint.name + "/velocity");
          command_data_[joint.name + "/velocity"] = 0.0;
        }
      }
    }

    // Declare state interfaces (encoder positions for each wheel)
    for (const auto & joint : info.joints) {
      for (const auto & state : joint.state_interfaces) {
        if (state.name == "position") {
          state_interfaces_.push_back(joint.name + "/position");
          state_data_[joint.name + "/position"] = 0.0;
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  // Step 2: Send commands to the Arduino
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    // Send velocity commands to the Arduino
    float fl_velocity = command_data_["front_left_wheel/velocity"];
    float fr_velocity = command_data_["front_right_wheel/velocity"];
    float rl_velocity = command_data_["rear_left_wheel/velocity"];
    float rr_velocity = command_data_["rear_right_wheel/velocity"];

    serial_port_.Write((char*)&fl_velocity);
    serial_port_.Write((char*)&fr_velocity);
    serial_port_.Write((char*)&rl_velocity);
    serial_port_.Write((char*)&rr_velocity);

    return hardware_interface::return_type::OK;
  }

  // Step 3: Read encoder data from the Arduino
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    // Read encoder data from the Arduino (if available)
    // float fl_position, fr_position, rl_position, rr_position;
    // serial_port_.Read((char*)&fl_position, sizeof(float));
    // serial_port_.Read((char*)&fr_position, sizeof(float));
    // serial_port_.Read((char*)&rl_position, sizeof(float));
    // serial_port_.Read((char*)&rr_position, sizeof(float));

    // Update state data
    // state_data_["front_left_wheel/position"] = fl_position;
    // state_data_["front_right_wheel/position"] = fr_position;
    // state_data_["rear_left_wheel/position"] = rl_position;
    // state_data_["rear_right_wheel/position"] = rr_position;

    return hardware_interface::return_type::OK;
  }

  // Step 4: Expose command interfaces
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (const auto & key : command_interfaces_) {
      command_interfaces.push_back(hardware_interface::CommandInterface(
        key, "velocity", &command_data_[key]));
    }
    return command_interfaces;
  }

  // Step 5: Expose state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const auto & key : state_interfaces_) {
      state_interfaces.push_back(hardware_interface::StateInterface(
        key, "position", &state_data_[key]));
    }
    return state_interfaces;
  }

  // Step 6: Other required methods (minimal implementation)
  hardware_interface::return_type start()
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop()
  {
    return hardware_interface::return_type::OK;
  }

private:
  LibSerial::SerialPort serial_port_;
  std::vector<std::string> command_interfaces_;
  std::vector<std::string> state_interfaces_;
  std::map<std::string, double> command_data_;
  std::map<std::string, double> state_data_;
};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ArduinoHardwareInterface, hardware_interface::SystemInterface)