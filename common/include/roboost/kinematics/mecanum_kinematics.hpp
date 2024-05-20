#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <array>
#include <memory>
#include <roboost/kinematics/base_kinematics.hpp>
#include <roboost/utils/matrices.hpp>

namespace roboost
{
    namespace kinematics
    {

        struct MecanumConfig
        {
            float wheel_radius;
            float wheel_base;
            float track_width;
        };

        struct MecanumKinematicState : public BaseKinematicState
        {
            std::vector<float> wheel_velocities;
        };

        class MecanumKinematics : public BaseKinematics
        {
        public:
            MecanumKinematics(const MecanumConfig& config) : config_(config)
            {
                const float l = (config_.wheel_base + config_.track_width) / 2.0;

                // clang-format off
                forward_kinematics_ = {{
                    {1, -1, -l},
                    {-1, -1, -l},
                    {1, 1, -l},
                    {-1, 1, -l}
                }};
                inverse_kinematics_ = {{
                    {1, -1, 1, -1},
                    {-1, -1, 1, 1},
                    {-1 / l, -1 / l, -1 / l, -1 / l}
                }};
                // clang-format on
            }

            std::unique_ptr<BaseKinematicState> calculate_robot_state(const std::vector<float>& wheel_velocities) override
            {
                roboost::math::Vector<4> wheel_velocities_arr;
                std::copy(wheel_velocities.begin(), wheel_velocities.end(), wheel_velocities_arr.begin());
                auto robot_velocity = roboost::math::multiply(inverse_kinematics_, wheel_velocities_arr);
                robot_velocity = roboost::math::scale(robot_velocity, config_.wheel_radius / 4.0f);
                auto state = std::make_unique<MecanumKinematicState>();
                state->linear_x = robot_velocity[0];
                state->linear_y = robot_velocity[1];
                state->angular_z = robot_velocity[2];
                state->wheel_velocities = wheel_velocities;
                return state;
            }

            std::vector<float> calculate_joint_states(const BaseKinematicState& robot_state) override
            {
                const auto* mecanum_state = static_cast<const MecanumKinematicState*>(&robot_state);
                roboost::math::Vector<3> robot_vel = {mecanum_state->linear_x, mecanum_state->linear_y, mecanum_state->angular_z};
                auto wheel_velocities = roboost::math::multiply(forward_kinematics_, robot_vel);
                wheel_velocities = roboost::math::scale(wheel_velocities, 1 / config_.wheel_radius);
                return {wheel_velocities[0], wheel_velocities[1], wheel_velocities[2], wheel_velocities[3]};
            }

        private:
            MecanumConfig config_;
            roboost::math::Matrix<4, 3> forward_kinematics_;
            roboost::math::Matrix<3, 4> inverse_kinematics_;
        };

    } // namespace kinematics
} // namespace roboost

#endif // MECANUM_KINEMATICS_H
