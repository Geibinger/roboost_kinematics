#ifndef BASE_KINEMATICS_H
#define BASE_KINEMATICS_H

#include <memory>
#include <vector>

namespace roboost
{
    namespace kinematics
    {

        struct BaseKinematicState : public description::State
        {
            float linear_x = 0.0f;
            float linear_y = 0.0f;
            float angular_z = 0.0f;

            std::string serialize() const override { return "linear_x: " + std::to_string(linear_x) + ", linear_y: " + std::to_string(linear_y) + ", angular_z: " + std::to_string(angular_z); }
        };

        class BaseKinematics
        {
        public:
            virtual ~BaseKinematics() = default;

            virtual std::unique_ptr<BaseKinematicState> calculate_robot_state(const std::vector<float>& inputs) = 0;
            virtual std::vector<float> calculate_joint_states(const BaseKinematicState& robot_state) = 0;
        };

    } // namespace kinematics
} // namespace roboost

#endif // BASE_KINEMATICS_H