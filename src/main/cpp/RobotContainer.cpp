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

// Autononous
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>

#include <frc/Filesystem.h>
#include <stdexcept>
#include <iostream>

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
    // NamedCommands::registerCommand
    // (
    //     "DriveForward", 
    //     std::move
    //         (frc2::FunctionalCommand 
    //             (
    //                 [this]{m_drive.ResetEncoders();}, // Initialize: reset encoders to 0 when starting autonomous
    //                 [this]{m_drive.Drive(3_mps, 0_mps, 0_rad_per_s, false);}, // Execute: Drive forward 3 meters per second
    //                 [this](bool interrupted){m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false);}, // End
    //                 [this]
    //                     {
    //                         units::meter_t flDist = m_drive.Get_FrontLeft_Encoder();
    //                         units::meter_t frDist  = m_drive.Get_FrontRight_Encoder();
    //                         units::meter_t rlDist = m_drive.Get_RearLeft_Encoder();
    //                         units::meter_t rrDist = m_drive.Get_RearRight_Encoder();

    //                         if (flDist >= 3_m and frDist >= 3_m and rlDist >= 3_m and rrDist >= 3_m)
    //                         {
    //                             return true;
    //                         }
    //                         else
    //                         {
    //                             return false;
    //                         }
    //                     }, // isfinished()

    //                     {&m_drive} // reference to the subsystem were using
    //             ).ToPtr()
    //         )
    //     ); // <- This example method returns CommandPtr},


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
  
 //Autonomous odometry using pathplanner

    using namespace pathplanner;

    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    
    return (PathPlannerAuto("Simple Auto").ToPtr().Unwrap().get());

} 
