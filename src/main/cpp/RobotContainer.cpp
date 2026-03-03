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

   pathplanner::NamedCommands::registerCommand("Deploy Arm", std::make_shared<ArmCmd>(&m_arm, &m_intake, ArmConstants::kDeployedPosition, IntakeConstants::kIntakePower));
   pathplanner::NamedCommands::registerCommand("Retract Arm", std::make_shared<ArmCmd>(&m_arm, &m_intake, ArmConstants::kStowedPosition, -IntakeConstants::kIntakePower));
   pathplanner::NamedCommands::registerCommand("Shoot", std::make_shared<ShootCmd>(&m_shooter, &m_tower, ShooterConstants::kShooterVelocity, TowerConstants::kTowerVelocity));
   pathplanner::NamedCommands::registerCommand("Intake", std::make_shared<LoadCmd>(&m_intake, IntakeConstants::kIntakeVelocity));
   pathplanner::NamedCommands::registerCommand("Intake Reverse", std::make_shared<LoadCmd>(&m_intake, -IntakeConstants::kIntakeVelocity));
   pathplanner::NamedCommands::registerCommand("Shoot Reverse", std::make_shared<LoadCmd>(&m_intake, -IntakeConstants::kIntakeVelocity));


    ConfigureButtonBindings();


    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
}

void RobotContainer::ConfigureButtonBindings() {
//   frc2::JoystickButton(&m_driverController,
//                        frc::XboxController::Button::kRightBumper)
//       .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

    
    m_driverController.RightBumper().WhileTrue(new frc2::InstantCommand([this]{m_drive.SetX(); }, {&m_drive}));

    // Arm Deploy
    m_driverController.A().OnTrue(ArmCmd(&m_arm, &m_intake, ArmConstants::kDeployedPosition, IntakeConstants::kIntakePower).ToPtr());
    // Arm Stow
    m_driverController.B().OnTrue(ArmCmd(&m_arm, &m_intake, ArmConstants::kStowedPosition, -IntakeConstants::kIntakePower).ToPtr());

    // Shoot
    (!m_driverController.LeftBumper()
    && m_driverController.RightTrigger()).WhileTrue(ShootCmd(&m_shooter, &m_tower, ShooterConstants::kShooterVelocity, TowerConstants::kTowerVelocity).ToPtr());
    // Shoot Reverse
    (m_driverController.LeftBumper()
    && m_driverController.RightTrigger()).WhileTrue(ShootCmd(&m_shooter, &m_tower, -ShooterConstants::kShooterVelocity, -TowerConstants::kTowerVelocity).ToPtr());

    // Intake
    (!m_driverController.LeftBumper()
    && m_driverController.LeftTrigger()).WhileTrue(LoadCmd(&m_intake, IntakeConstants::kIntakeVelocity).ToPtr());

    // Intake Reverse
    (m_driverController.LeftBumper()
    && m_driverController.LeftTrigger()).WhileTrue(LoadCmd(&m_intake, -IntakeConstants::kIntakeVelocity).ToPtr());

}


frc2::Command* RobotContainer::GetAutonomousCommand() 
{

    // // An example trajectory to follow.  All units in meters.
    // std::string filepath = "Simple Auto";
    // return pathplanner::PathPlannerAuto(filepath).ToPtr();

    return m_autoChooser.GetSelected();
}
