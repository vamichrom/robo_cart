// Copyright (c) 2025, vamichrom
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "ros2_control_arduino_hw/robo_cart_arduino_hw.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_arduino_hw
{
  hardware_interface::CallbackReturn RoboCartArduinoHW::on_init(
      const hardware_interface::HardwareInfo &info) // info of HardwareInfo class contains all the parameters set in xacro file from which this hw interface is called
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Get the joint names from the hardware info
    cfg_.joint_names.emplace_back(info_.hardware_parameters["front_left_joint_name"]);
    cfg_.joint_names.emplace_back(info_.hardware_parameters["front_right_joint_name"]);
    cfg_.joint_names.emplace_back(info_.hardware_parameters["back_left_joint_name"]);
    cfg_.joint_names.emplace_back(info_.hardware_parameters["back_right_joint_name"]);
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Loop rate: %f", cfg_.loop_rate);
    cfg_.device = info_.hardware_parameters["device"];
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Device = %s", cfg_.device.c_str());
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Baud rate: %d", cfg_.baud_rate);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Timeout: %d", cfg_.timeout_ms);
    cfg_.encoder_ticks_per_rev = std::stoi(info_.hardware_parameters["encoder_counts_per_revolution"]);
    ;
    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "PID values not supplied, using defaults.");
    }

    // Configure the wheels
    for (int i = 0; i < 4; i++)
    {
      wheels_.emplace_back();
      RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Setting up Wheel[%d]", i);
      wheels_[i].setup(cfg_.joint_names[i], cfg_.encoder_ticks_per_rev);
      RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Set up Wheel[%d] with name %s and rads_per_count %f", i, wheels_[i].name.c_str(), wheels_[i].rads_per_count);
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "RoboCartArduinoHW initialized successfully");

    // Looping over the joints to assert that they make sense for this robot (ie. Velocity command interface, and Position and Velocity state interfaces)
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // RoboCartArduinoHW has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RoboCartArduinoHW"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RoboCartArduinoHW"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RoboCartArduinoHW"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RoboCartArduinoHW"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RoboCartArduinoHW"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoboCartArduinoHW::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Configuring ...please wait...");
    do
    {
      try
      {
        comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RoboCartArduinoHW"), "Failed to connect to arduino: %s", e.what());
      }
    } while (!comms_.connected());
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoboCartArduinoHW::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Cleaning up ...please wait...");
    // Stop hardware communication
    if (comms_.connected())
      comms_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RoboCartArduinoHW::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels_[i].pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels_[i].vel));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RoboCartArduinoHW::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels_[i].cmd));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn RoboCartArduinoHW::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Activating ...please wait...");
    if (cfg_.pid_p > 0)
    {
      comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
    }
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Successfully activated!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RoboCartArduinoHW::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Deactivating ...please wait...");
    comms_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RoboCartArduinoHW::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (!comms_.connected())
      return hardware_interface::return_type::ERROR;

    // Read encoder values to get the current position and velocity of the wheels
    comms_.read_encoder_values(wheels_);

    // To get the velocity, we store the previous position and measure the change in position in the measured duration of time between them
    double prev_positions[4]; // could be changed to float for efficiency: 4 bytes instead of 8 bytes per value
    for (int i = 0; i < 4; i++)
      prev_positions[i] = wheels_[i].pos;

    // Get the current position of the wheels and store it in the pos variable ==> pos = encoder counts * radians per encoder count
    for (int i = 0; i < 4; i++)
    {
      wheels_[i].pos = wheels_[i].getEncoderAngle(); // Encoder counts are converted to radians by multiplying by the number of radians per encoder count
    }

    // Calculate the velocity

    for (int i = 0; i < 4; i++)
    {
      wheels_[i].vel = (wheels_[i].pos - prev_positions[i]) / period.seconds(); // Velocity is calculated by dividing the change in position by the time elapsed
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RoboCartArduinoHW::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
      return hardware_interface::return_type::ERROR;

    int motor_counts_per_loop[4];
    /*
    The motor_counts_per_loop array stores the number of encoder counts that each wheel should rotate in one loop cycle.
    This is calculated by dividing the desired angular velocity of the wheel (wheels_[i].cmd) by the number of radians per encoder count (wheels_[i].rads_per_count) and then by the loop rate (cfg_.loop_rate).
    This gives the number of encoder counts per second, which is then multiplied by the loop duration to get the number of counts per loop.
    */

    for (int i = 0; i < 4; i++)
    {
      //RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Wheel[%d] Ang_vel=%f, Rads_per_count=%f, Loop_rate=%f ", i, wheels_[i].cmd, wheels_[i].rads_per_count, cfg_.loop_rate);
      motor_counts_per_loop[i] = wheels_[i].cmd/wheels_[i].rads_per_count/cfg_.loop_rate;
      //RCLCPP_INFO(rclcpp::get_logger("RoboCartArduinoHW"), "Set up motor_counts_per_loop[%d] with %d", i, motor_counts_per_loop[i]);
    }
    comms_.set_motor_values(motor_counts_per_loop);

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_arduino_hw

#include "pluginlib/class_list_macros.hpp"

// Telling how to export the plugin
PLUGINLIB_EXPORT_CLASS(
    ros2_control_arduino_hw::RoboCartArduinoHW, hardware_interface::SystemInterface)
