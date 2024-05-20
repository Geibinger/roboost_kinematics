#ifndef COMBINED_KINEMATICS_H
#define COMBINED_KINEMATICS_H

#include <cmath>
#include <roboost/kinematics/base_kinematics.hpp>
#include <roboost/kinematics/manipulator_kinematics.hpp>

namespace roboost
{
    namespace kinematics
    {

        struct CombinedKinematicState
        {
            BaseKinematicState base_state;
            ManipulatorKinematicState manipulator_state;
        };

        ManipulatorKinematicState combine_states(const BaseKinematicState& base_state, const ManipulatorKinematicState& manipulator_state)
        {
            float x = base_state.linear_x + manipulator_state.x * cos(base_state.angular_z) - manipulator_state.y * sin(base_state.angular_z);
            float y = base_state.linear_y + manipulator_state.x * sin(base_state.angular_z) + manipulator_state.y * cos(base_state.angular_z);
            float theta = base_state.angular_z + manipulator_state.theta;
            ManipulatorKinematicState combined_state;
            combined_state.x = x;
            combined_state.y = y;
            combined_state.theta = theta;
            return combined_state;
        }

    } // namespace kinematics
} // namespace roboost

#endif // COMBINED_KINEMATICS_H
