#ifndef DIFFERENTIAL_DRIVE_KINEMATICS_H
#define DIFFERENTIAL_DRIVE_KINEMATICS_H

#include <roboost/kinematics/base_kinematics.hpp>

namespace roboost
{
    namespace kinematics
    {

        struct DifferentialConfig
        {
            float wheel_radius;
            float wheel_base;
        };

        struct DifferentialKinematicState : public BaseKinematicState
        {
            float v_left = 0.0f;
            float v_right = 0.0f;
        };

        class DifferentialDriveKinematics : public BaseKinematics
        {
        public:
            DifferentialDriveKinematics(const DifferentialConfig& config) : config_(config) {}

            std::unique_ptr<BaseKinematicState> calculate_robot_state(const std::vector<float>& inputs) override
            {
                float v_left = inputs[0];
                float v_right = inputs[1];
                float v = (v_left + v_right) / 2.0;
                float omega = (v_right - v_left) / config_.wheel_base;
                auto state = std::make_unique<DifferentialKinematicState>();
                state->linear_x = v;
                state->angular_z = omega;
                state->v_left = v_left;
                state->v_right = v_right;
                return state;
            }

            std::vector<float> calculate_joint_states(const BaseKinematicState& robot_state) override
            {
                const auto* diff_state = static_cast<const DifferentialKinematicState*>(&robot_state);
                return {diff_state->v_left, diff_state->v_right};
            }

        private:
            DifferentialConfig config_;
        };

    } // namespace kinematics
} // namespace roboost

#endif // DIFFERENTIAL_DRIVE_KINEMATICS_H