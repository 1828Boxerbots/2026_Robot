// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    0.596_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.596_m;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
// CAN IDs ARE NOT FOR THE COMP ROBOT
constexpr int kFrontLeftDrivingCanId = 10;
constexpr int kRearLeftDrivingCanId = 8;
constexpr int kFrontRightDrivingCanId = 2;
constexpr int kRearRightDrivingCanId = 4;

constexpr int kFrontLeftTurningCanId = 9;
constexpr int kRearLeftTurningCanId = 7;
constexpr int kFrontRightTurningCanId = 3;
constexpr int kRearRightTurningCanId = 5;
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;





}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

namespace IntakeConstants
{
    // Ports
    inline constexpr int kIntakeMotorPort = 11;

    // Speeds
    inline constexpr int kIntakePower = 0.5;

    inline constexpr double kWheelDiameter = 0.0508; // value is in meters
    inline constexpr double kMotorReduction = 1.0;

} // namespace IntakeConstants

namespace ArmConstants
{
    // Ports
    inline constexpr int kArmMotorAPort = 12;
    inline constexpr int kArmMotorBPort = 13;

    // Encoder Values in Degrees
    inline constexpr int kStowedPosition = 0;
    inline constexpr int kDeloyedPositon = 130; // 13pi/18 in rad

    // Other Konstants
    inline constexpr double kPositionTolerance = 20.0;

} // namespace ArmConstants

namespace TowerConstants
{
    //  Ports
    inline constexpr int kTowerMotorPort = 14;
    
    // Other Knostants
    inline constexpr double kPower = 0.5;
    
    inline constexpr double kWheelDiameter = 0.0508; // value is in meters
    inline constexpr double kMotorReduction = 1.0;

} // namespace TowerConstants

namespace ShooterConstants
{
    //  Ports
    inline constexpr int kShooterMotorAPort = 15;
    inline constexpr int kShooterMotorBPort = 16;
    
    // Other Knostants
    inline constexpr double kVelocity = 5.0;
    
    inline constexpr double kWheelDiameter = 0.1016; // value is in meters
    inline constexpr double kMotorReduction = 1.0;
    
} // namespace ShooterConstants

namespace OtherConstants
{
    inline constexpr double kNeo2FeedForwardRps = 5676.0 / 60;
    
} // namespce OtherConstants

namespace VisionConstants
{
    inline constexpr double kTagTrackingMultipler = 1.0; // changes how agressive it rotated to the target

    inline constexpr double kShooterHeight = 0.5207; // In meters
    inline constexpr double kHubHeight = 1.8288; // In meters
    inline constexpr double kGravity = 9.81; // In meters per second
    inline constexpr double kLaunchAngle = (79.0 * (3.14 / 180)); // In degree
    
} // namespace VisionConstants
