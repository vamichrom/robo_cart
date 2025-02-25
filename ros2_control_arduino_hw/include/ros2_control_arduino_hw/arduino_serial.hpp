#ifndef ROS2_CONTROL_ARDUINO_HW__ARDUINO_SERIAL_HPP
#define ROS2_CONTROL_ARDUINO_HW__ARDUINO_SERIAL_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include "ros2_control_arduino_hw/wheel.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

namespace ros2_control_arduino_hw
{
  class ArduinoSerial
  {
  public:
    ArduinoSerial()
    {
      // Default constructor
    }

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
      timeout_ms_ = timeout_ms;
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect()
    {
      serial_conn_.Close();
    }

    bool connected() const
    {
      return serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = true)
    {
      serial_conn_.FlushIOBuffers(); // Just in case
      serial_conn_.Write(msg_to_send);

      std::string response = "";
      try
      {
        // Responses end with \r\n so we will read up to (and including) the \n.
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
      }
      catch (const LibSerial::ReadTimeout &)
      {
        std::cerr << "The ReadByte() call has timed out." << std::endl;
      }

      if (print_output)
      {
        std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
      }

      return response;
    }

    void send_empty_msg()
    {
      std::string response = send_msg("\r");
    }

    void read_encoder_values(std::vector<Wheel> &wheels)
    {
      std::string response = send_msg("e\r");

      std::string delimiter = " ";
      std::string token_arr[4];
      size_t pos = 0; // Check for possible error. It seems it should be the length of the first message.
      int i = 0;
      // Split the response into tokens
      // Find delimiter and split the string at it, erase the substring+delimiter from the response, repeat
      while (pos != std::string::npos && i < 3)
      {
        token_arr[i] = response.substr(0, pos); // Read pos characters from the beginning (index 0)
        response.erase(0, pos + delimiter.length());
        pos = response.find(delimiter); // Returns std::string::npos if it fails
        i++;
      }
      token_arr[3] = response;

      for (int i = 0; i < 4; i++)
        wheels[i].enc = std::atoi(token_arr[i].c_str());
    }

    void set_motor_values(int *motor_values)
    {
      std::stringstream ss;
      ss << "m " << motor_values[0] << " " << motor_values[1] << " " << motor_values[2] << " " << motor_values[3] << "\r";
      send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
      std::stringstream ss;
      ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
      send_msg(ss.str());
    }

  private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
  };
} // namespace ros2_control_arduino_hw

#endif // ROS2_CONTROL_ARDUINO_HW__ARDUINO_SERIAL_HPP