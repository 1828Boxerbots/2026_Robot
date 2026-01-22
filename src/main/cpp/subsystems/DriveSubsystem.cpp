// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  // Usage reporting for MAXSwerve template
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
             
//Configuring the Autobuilder for PathPlanner
        
  units::meters_per_second_t xSpeed{0_mps};
  units::meters_per_second_t ySpeed{0_mps};
  units::radians_per_second_t rot{0_rad_per_s};

  
  RobotConfig config = RobotConfig::fromGUISettings();

   // Configure the AutoBuilder last
    AutoBuilder::configure(
        // Robot pose supplier  
        [this]()
        { 
          return GetPose(); 
        },
        // Method to reset odometry (will be called if your auto has a starting pose)
        [this](const frc::Pose2d& pose)
        { 
          ResetOdometry(pose); 
        },
         // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [
          this,
          xSpeed,
          ySpeed,
          rot
        ]
        (
          // units::meters_per_second_t xSpeed, 
          // units::meters_per_second_t ySpeed, 
          // units::radians_per_second_t rot
        )
        { 
          return GetChassisSpeeds(xSpeed, ySpeed, rot);
        },
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        [this](const frc::ChassisSpeeds& speeds)
        { 
          units::meters_per_second_t xSpeed = speeds.vx;
          units::meters_per_second_t ySpeed = speeds.vy;
          units::radians_per_second_t rot = speeds.omega;

          Drive(xSpeed, ySpeed, rot, false); 
        },
        // PPHolonomicController is the built in path following controller for holonomic drive trains
        std::make_shared<PPHolonomicDriveController>
        ( 
            AutoConstants::translationConstants, // Translation PID constants
            AutoConstants::rotationConstants // Rotation PID constants
        ),
        // The robot configuration
        config,
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE 
        []() 
        {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) 
                return alliance.value() == frc::DriverStation::Alliance::kRed;
                
            return false;
        },
        // Reference to this subsystem to set requirements
        this 
    );
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(units::radian_t{
                        m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(
             units::radian_t{m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
