#ifndef MANIPULATOR_KINEMATICS_H
#define MANIPULATOR_KINEMATICS_H

#include <memory>
#include <vector>

namespace roboost
{
    namespace kinematics
    {

        struct ManipulatorKinematicState
        {
            float x = 0.0f;
            float y = 0.0f;
            float theta = 0.0f;

            virtual ~ManipulatorKinematicState() = default;
        };

        class ManipulatorKinematics
        {
        public:
            virtual ~ManipulatorKinematics() = default;

            virtual std::unique_ptr<ManipulatorKinematicState> calculate_end_effector_position(const std::vector<float>& joint_angles) = 0;
            virtual std::vector<float> calculate_joint_angles(const ManipulatorKinematicState& end_effector_position) = 0;
        };

    } // namespace kinematics
} // namespace roboost

#endif // MANIPULATOR_KINEMATICS_H
