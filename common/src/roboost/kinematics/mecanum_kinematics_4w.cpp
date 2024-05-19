/**
 * @file mecanum_kinematics_4w.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of MecanumKinematics4W class.
 * @version 1.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <roboost/kinematics/kinematics.hpp>

// TODO: Remove Eigen dependency -> use std::array instead

using namespace roboost::kinematics;
using namespace roboost::math;

MecanumKinematics4W::MecanumKinematics4W(float wheel_radius, float wheel_base, float track_width) : wheel_radius_(wheel_radius), wheel_base_(wheel_base), track_width_(track_width)
{
    const float l = (wheel_base_ + track_width_) / 2.0;
    // clang-format off
    forward_kinematics_ = { {1, -1, -l},
                            {-1, -1, -l},
                            {1, 1, -l},
                            {-1, 1, -l}};

    inverse_kinematics_ = { {1, -1, 1, -1},
                            {-1, -1, 1, 1},
                            {-1 / l, -1 / l, -1 / l, -1 / l}};
    // clang-format on
}

Vector<float> MecanumKinematics4W::calculate_wheel_velocity(const Vector<float>& robot_velocity)
{
    Vector<float> wheel_velocity = forward_kinematics_ * robot_velocity;
    return wheel_velocity * (1 / wheel_radius_);
}

Vector<float> MecanumKinematics4W::calculate_robot_velocity(const Vector<float>& wheel_velocity)
{
    Vector<float> robot_velocity = inverse_kinematics_ * wheel_velocity;
    return robot_velocity * (wheel_radius_ / 4.0f);
}