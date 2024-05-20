#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <cmath>
#include <memory>
#include <roboost/kinematics/manipulator_kinematics.hpp>
#include <vector>

namespace roboost
{
    namespace kinematics
    {

        struct ArmConfig
        {
            std::vector<float> link_lengths;
        };

        struct ArmKinematicState : public ManipulatorKinematicState
        {
            std::vector<float> joint_angles;
        };

        class ArmKinematics : public ManipulatorKinematics
        {
        public:
            ArmKinematics(const ArmConfig& config) : config_(config) {}

            std::unique_ptr<ManipulatorKinematicState> calculate_end_effector_position(const std::vector<float>& joint_angles) override
            {
                float x = 0.0f;
                float y = 0.0f;
                float total_angle = 0.0f;

                for (size_t i = 0; i < config_.link_lengths.size(); ++i)
                {
                    total_angle += joint_angles[i];
                    x += config_.link_lengths[i] * cos(total_angle);
                    y += config_.link_lengths[i] * sin(total_angle);
                }

                auto state = std::make_unique<ArmKinematicState>();
                state->x = x;
                state->y = y;
                state->theta = total_angle;
                state->joint_angles = joint_angles;

                return state;
            }

            std::vector<float> calculate_joint_angles(const ManipulatorKinematicState& end_effector_position) override
            {
                std::vector<float> joint_angles(config_.link_lengths.size(), 0.0f);
                float x = end_effector_position.x;
                float y = end_effector_position.y;
                float L1 = config_.link_lengths[0];
                float L2 = config_.link_lengths[1];

                float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
                float theta2 = atan2(sqrt(1 - D * D), D);

                float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

                joint_angles[0] = theta1;
                joint_angles[1] = theta2;

                if (config_.link_lengths.size() > 2)
                {
                    joint_angles[2] = end_effector_position.theta - theta1 - theta2;
                }

                return joint_angles;
            }

        private:
            ArmConfig config_;
        };

    } // namespace kinematics
} // namespace roboost

#endif // ARM_KINEMATICS_H
