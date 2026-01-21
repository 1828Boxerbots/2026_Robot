// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/length.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
    NamedCommands::registerCommand
    (
        "DriveForward", 
        std::move
            (frc2::FunctionalCommand 
                (
                    [this]{m_drive.ResetEncoders();}, // Initialize: reset encoders to 0 when starting autonomous
                    [this]{m_drive.Drive(3_mps, 0_mps, 0_rad_per_s, false);}, // Execute: Drive forward 3 meters per second
                    [this](bool interrupted){m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);}, // End
                    [this]
                        {
                            units::meter_t flDist = m_drive.Get_FrontLeft_Encoder();
                            units::meter_t frDist  = m_drive.Get_FrontRight_Encoder();
                            units::meter_t rlDist = m_drive.Get_RearLeft_Encoder();
                            units::meter_t rrDist = m_drive.Get_RearRight_Encoder();

                            if (flDist >= 3_m and frDist >= 3_m and rlDist >= 3_m and rrDist >= 3_m)
                            {
                                return true;
                            }
                            else
                            {
                                return false;
                            }
                        }, // isfinished()

                        {&m_drive} // reference to the subsystem were using
                ).ToPtr()
            )
        ); // <- This example method returns CommandPtr},


  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());


  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));

 //Autonomous odometry using pathplanner

    using namespace pathplanner;

    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    
    return (PathPlannerAuto("Simple Auto").ToPtr().Unwrap().get());

} 
