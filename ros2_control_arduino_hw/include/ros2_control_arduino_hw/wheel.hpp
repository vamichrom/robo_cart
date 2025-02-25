#ifndef ROS2_CONTROL_ARDUINO_HW__WHEEL_HPP
#define ROS2_CONTROL_ARDUINO_HW__WHEEL_HPP

#include <string>
#include <cmath>
// TODO: Change all doubles to floats ??? 8 bytes vs 4 bytes per value
class Wheel
{
public:
    // Default constructor
    Wheel() : name(""), enc(0), vel(0.0), pos(0.0), cmd(0.0), rads_per_count(0.0) {}

    // Parameterized constructor
    Wheel(const std::string &wheel_name, int counts_per_rev) : enc(0), vel(0.0), pos(0.0), cmd(0.0)
    {
        setup(wheel_name, counts_per_rev);
    }
    std::string name;      // wheel name
    int enc;               // encoder value
    double vel;            // velocity (state: angular velocity in radians per second)
    double pos;            // position (state: angle in radians)
    double cmd;            // command value (command: velocity)
    double rads_per_count; // conversion factor from encoder ticks to radians

    void setup(const std::string &wheel_name, int counts_per_rev)
    {
        name = wheel_name;
        rads_per_count = 2 * M_PI / counts_per_rev;
    }

    double getEncoderAngle() const
    {
        return enc * rads_per_count;
    }
};

#endif // ROS2_CONTROL_ARDUINO_HW__WHEEL_HPP
