#ifndef ROS2_CONTROL_ARDUINO_HW__ARDUINO_SERIAL_HPP
#define ROS2_CONTROL_ARDUINO_HW__ARDUINO_SERIAL_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include "ros2_control_arduino_hw/wheel.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate);

class ArduinoSerial
{
public:
  ArduinoSerial();
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void disconnect();
  bool connected() const;
  std::string send_msg(const std::string &msg_to_send, bool print_output = false);
  void send_empty_msg();
  void read_encoder_values(std::vector<Wheel> &wheels);
  void set_motor_values(int *motor_values);
  void set_pid_values(int k_p, int k_d, int k_i, int k_o);

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // ROS2_CONTROL_ARDUINO_HW__ARDUINO_SERIAL_HPP