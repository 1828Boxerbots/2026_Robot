// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ResetOdometryCmd.h"

ResetOdometryCmd::ResetOdometryCmd(DriveSubsystem *driveSub) 
{

  m_isFinished = true;
  m_driveSub = driveSub;
  AddRequirements(m_driveSub);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ResetOdometryCmd::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void ResetOdometryCmd::Execute()
{

}

// Called once the command ends or is interrupted.
void ResetOdometryCmd::End(bool interrupted)
{
  m_driveSub->ResetOdometry(frc::Pose2d(0_m, 0_m, 0_deg));
}

// Returns true when the command should end.
bool ResetOdometryCmd::IsFinished() 
{
  return m_isFinished;
}
