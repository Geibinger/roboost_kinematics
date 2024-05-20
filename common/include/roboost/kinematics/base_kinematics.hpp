#ifndef BASE_KINEMATICS_H
#define BASE_KINEMATICS_H

#include <memory>
#include <vector>

namespace roboost
{
    namespace kinematics
    {

        struct BaseKinematicState
        {
            float linear_x = 0.0f;
            float linear_y = 0.0f;
            float angular_z = 0.0f;

            virtual ~BaseKinematicState() = default;
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