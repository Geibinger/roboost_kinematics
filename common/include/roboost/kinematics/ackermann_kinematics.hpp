#ifndef ACKERMANN_KINEMATICS_H
#define ACKERMANN_KINEMATICS_H

#include <cmath>
#include <memory>
#include <roboost/kinematics/base_kinematics.hpp>
#include <vector>

namespace roboost
{
    namespace kinematics
    {

        struct AckermannConfig
        {
            float wheel_base;
            float track_width;
        };

        struct AckermannKinematicState : public BaseKinematicState
        {
            float steering_angle = 0.0f;
        };

        class AckermannKinematics : public BaseKinematics
        {
        public:
            AckermannKinematics(const AckermannConfig& config) : config_(config) {}

            std::unique_ptr<BaseKinematicState> calculate_robot_state(const std::vector<float>& inputs) override
            {
                float v = inputs[0];
                float steering_angle = inputs[1];
                float omega = v * tan(steering_angle) / config_.wheel_base;
                auto state = std::make_unique<AckermannKinematicState>();
                state->linear_x = v;
                state->angular_z = omega;
                state->steering_angle = steering_angle;
                return state;
            }

            std::vector<float> calculate_joint_states(const BaseKinematicState& robot_state) override
            {
                const auto* ackermann_state = static_cast<const AckermannKinematicState*>(&robot_state);
                return {ackermann_state->linear_x, ackermann_state->steering_angle};
            }

        private:
            AckermannConfig config_;
        };

    } // namespace kinematics
} // namespace roboost

#endif // ACKERMANN_KINEMATICS_H