#ifndef ROS2_CONTROL_ARDUINO_HW__WHEEL_HPP
#define ROS2_CONTROL_ARDUINO_HW__WHEEL_HPP

#include <string>
#include <cmath>
// TODO: Change all doubles to floats ??? 8 bytes vs 4 bytes per value
class Wheel
{
public:
    Wheel();
    Wheel(const std::string &name, int counts_per_rev);
    std::string name;                                        // wheel name
    int enc;                                                 // encoder value
    double vel;                                              // velocity (state: angular velocity in radians per second)
    double pos;                                              // position (state: angle in radians)
    double cmd;                                              // command value (command: velocity)
    double rads_per_count;                                   // conversion factor from encoder ticks to radians
    void setup(const std::string &wheel_name, int counts_per_rev); // setup function to set name and conversion factor
    double getEncoderAngle() const;
};

#endif // ROS2_CONTROL_ARDUINO_HW__WHEEL_HPP
