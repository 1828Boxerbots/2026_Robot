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

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/ArmCmd.h"
#include "commands/LoadCmd.h"
#include "commands/ShootCmd.h"

// Autononous
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>

#include <frc/Filesystem.h>
#include <stdexcept>
#include <iostream>

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

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
//   frc2::JoystickButton(&m_driverController,
//                        frc::XboxController::Button::kRightBumper)
//       .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

    m_driverController.RightBumper().WhileTrue(new frc2::InstantCommand([this]{m_drive.SetX(); }, {&m_drive}));

    // Arm Deploy
    m_driverController.A().OnTrue(ArmCmd(&m_arm, ArmConstants::kDeloyedPositon).ToPtr());
    // Arm Stow
    m_driverController.B().OnTrue(ArmCmd(&m_arm, ArmConstants::kStowedPosition).ToPtr());

    // Shoot
    (!m_driverController.LeftBumper()
    && m_driverController.RightTrigger()).WhileTrue(ShootCmd(&m_shooter, &m_tower, ShooterConstants::kVelocity, TowerConstants::kVelocity).ToPtr());
    // Shoot Reverse
    (m_driverController.LeftBumper()
    && m_driverController.RightTrigger()).WhileTrue(ShootCmd(&m_shooter, &m_tower, -ShooterConstants::kVelocity, -TowerConstants::kVelocity).ToPtr());

    // Intake
    (!m_driverController.LeftBumper()
    && m_driverController.LeftTrigger()).WhileTrue(LoadCmd(&m_intake, IntakeConstants::kVelocity).ToPtr());

    // Intake Reverse
    (m_driverController.LeftBumper()
    && m_driverController.LeftTrigger()).WhileTrue(LoadCmd(&m_intake, -IntakeConstants::kVelocity).ToPtr());

}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
    try
    {
        pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

        // Configure the AutoBuilder last
        pathplanner::AutoBuilder::configure(
            // Robot pose supplier  
            [this]()
            {
                return m_drive.GetPose();
            },
            // Method to reset odometry (will be called if your auto has a starting pose)
            [this](const frc::Pose2d& pose)
            { 
                m_drive.ResetOdometry(pose); 
            },
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            [this]()
            {
                return m_drive.GetRelativeChassisSpeeds();
            },
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            [this](const frc::ChassisSpeeds& speeds)
            { 
                units::meters_per_second_t xSpeed = speeds.vx;
                units::meters_per_second_t ySpeed = speeds.vy;
                units::radians_per_second_t rot = speeds.omega;

                m_drive.Drive(xSpeed, ySpeed, rot, false); 
            },
            // PPHolonomicController is the built in path following controller for holonomic drive trains
            std::make_shared<pathplanner::PPHolonomicDriveController>
            ( 
                pathplanner::PIDConstants(0.04, 0.0, 0.0), // Translation PID constants
                pathplanner::PIDConstants(1, 0.0, 0.0) // Rotation PID constants
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
            &m_drive
        );        

        // An example trajectory to follow.  All units in meters.
        std::string filepath = "Simple Auto";
        return pathplanner::PathPlannerAuto(filepath).ToPtr();
    }
    catch(std::exception& e) 
    {
        std::cout << e.what();
    }
}
