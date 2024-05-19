/**
 * @file kinematics.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Kinematics base class definitions.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <roboost/utils/matrices.hpp>

namespace roboost
{
    namespace kinematics
    {
        /**
         * @brief Abstract base class for defining kinematics calculations.
         *
         * This class provides an interface for calculating robot velocity and wheel
         * velocity based on the kinematic properties of the robot.
         */
        class Kinematics
        {
        public:
            /**
             * @brief Calculate robot velocity based on wheel velocities.
             *
             * @param wheel_velocity The velocities of individual wheels.
             * @return Vector The calculated robot velocity.
             */
            virtual roboost::math::Vector<float> calculate_robot_velocity(const roboost::math::Vector<float>& wheel_velocity) = 0;

            /**
             * @brief Calculate wheel velocities based on robot velocity.
             *
             * @param robot_velocity The velocity of the robot.
             * @return Vector The calculated wheel velocities.
             */
            virtual roboost::math::Vector<float> calculate_wheel_velocity(const roboost::math::Vector<float>& robot_velocity) = 0;
        };

        /**
         * @brief Mecanum kinematics for a 4-wheel robot.
         *
         * This class implements the kinematics calculations for a 4-wheel mecanum drive
         * robot. It takes into account the wheel radius, wheel base, and track width of
         * the robot.
         */
        class MecanumKinematics4W : public Kinematics
        {
        public:
            /**
             * @brief Construct a new Mecanum Kinematics 4W object with given
             * parameters.
             *
             * @param wheel_radius The radius of the wheels.
             * @param wheel_base The distance between wheel contact points in the x
             * direction.
             * @param track_width The distance between wheel contact points in the y
             * direction.
             */
            MecanumKinematics4W(float wheel_radius, float wheel_base, float track_width);

            /**
             * @brief Calculate robot velocity based on wheel velocities.
             *
             * @param wheel_velocity The velocities of individual wheels.
             * @return Vector The calculated robot velocity.
             */
            roboost::math::Vector<float> calculate_robot_velocity(const roboost::math::Vector<float>& wheel_velocity) override;

            /**
             * @brief Calculate wheel velocities based on robot velocity.
             *
             * @param robot_velocity The velocity of the robot.
             * @return Vector The calculated wheel velocities.
             */
            roboost::math::Vector<float> calculate_wheel_velocity(const roboost::math::Vector<float>& robot_velocity) override;

        private:
            float wheel_radius_; // Radius of the wheels.
            float wheel_base_;   // Distance between wheel contact points in the x direction.
            float track_width_;  // Distance between wheel contact points in the y direction.

            roboost::math::Matrix<float> forward_kinematics_; // Forward kinematics matrix
            roboost::math::Matrix<float> inverse_kinematics_; // Inverse kinematics matrix
        };

        // TODO: Implement DifferentialDriveKinematics class
        // TODO: Implement AckermannDriveKinematics class

    } // namespace kinematics
} // namespace roboost

#endif // KINEMATICS_H
