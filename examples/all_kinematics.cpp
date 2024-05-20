#include <Arduino.h>
#include <roboost/kinematics/ackermann_kinematics.hpp>
#include <roboost/kinematics/arm_kinematics.hpp>
#include <roboost/kinematics/combined_kinematics.hpp>
#include <roboost/kinematics/diff_drive_kinematics.hpp>
#include <roboost/kinematics/mecanum_kinematics.hpp>

void setup()
{
    Serial.begin(115200);

    // Initialize configurations
    roboost::kinematics::DifferentialConfig differential_config = {0.05, 0.3};
    roboost::kinematics::ArmConfig arm_config = {{0.3, 0.25, 0.2}};
    roboost::kinematics::MecanumConfig mecanum_config = {0.05, 0.2, 0.3};
    roboost::kinematics::AckermannConfig ackermann_config = {0.25, 0.3};

    // Initialize kinematics models
    roboost::kinematics::DifferentialDriveKinematics differential_kinematics(differential_config);
    roboost::kinematics::ArmKinematics arm_kinematics(arm_config);
    roboost::kinematics::MecanumKinematics mecanum_kinematics(mecanum_config);
    roboost::kinematics::AckermannKinematics ackermann_kinematics(ackermann_config);

    // Example inputs for Differential Drive Kinematics
    std::vector<float> differential_inputs = {0.5, 0.7};
    auto robot_velocity_differential = differential_kinematics.calculate_robot_state(differential_inputs);
    Serial.println("Differential Drive Robot Velocity:");
    Serial.println(robot_velocity_differential->linear_x);
    Serial.println(robot_velocity_differential->angular_z);

    // Example joint angles for Arm Kinematics
    std::vector<float> joint_angles = {0.1, 0.2, 0.3};
    auto end_effector_position = arm_kinematics.calculate_end_effector_position(joint_angles);
    Serial.println("Arm End-Effector Position:");
    Serial.println(end_effector_position->x);
    Serial.println(end_effector_position->y);
    Serial.println(end_effector_position->theta);

    // Example wheel velocities for Mecanum Kinematics
    std::vector<float> wheel_velocities_mecanum = {0.5, 0.5, 0.5, 0.5};
    auto robot_velocity_mecanum = mecanum_kinematics.calculate_robot_state(wheel_velocities_mecanum);
    Serial.println("Mecanum Robot Velocity:");
    Serial.println(robot_velocity_mecanum->linear_x);
    Serial.println(robot_velocity_mecanum->linear_y);
    Serial.println(robot_velocity_mecanum->angular_z);

    // Example inputs for Ackermann Kinematics
    std::vector<float> ackermann_inputs = {1.0, 0.1};
    auto robot_velocity_ackermann = ackermann_kinematics.calculate_robot_state(ackermann_inputs);
    Serial.println("Ackermann Robot Velocity:");
    Serial.println(robot_velocity_ackermann->linear_x);
    Serial.println(robot_velocity_ackermann->angular_z);

    // Example combined kinematics
    auto combined_position = roboost::kinematics::combine_states(*robot_velocity_differential, *end_effector_position);
    Serial.println("Combined End-Effector Position:");
    Serial.println(combined_position.x);
    Serial.println(combined_position.y);
    Serial.println(combined_position.theta);
}

void loop()
{
    // Your main code here
}
