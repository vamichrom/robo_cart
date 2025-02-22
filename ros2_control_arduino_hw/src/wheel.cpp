#include "ros2_control_arduino_hw/wheel.hpp"

// Default constructor
Wheel::Wheel() : name(""), enc(0), vel(0.0), pos(0.0), cmd(0.0), rads_per_count(0.0) {}

// Parameterized constructor
Wheel::Wheel(const std::string &wheel_name, int counts_per_rev) : enc(0), vel(0.0), pos(0.0), cmd(0.0)
{
    setup(wheel_name, counts_per_rev);
}

void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
    name = wheel_name;
    rads_per_count = 2 * M_PI / counts_per_rev;
}

double Wheel::getEncoderAngle() const
{
    return enc * rads_per_count;
}