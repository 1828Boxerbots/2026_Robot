
#include "commands/SetXCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

SetXCmd::SetXCmd(DriveSubsystem *driveSub)
{
  m_driveSub = driveSub;
  AddRequirements(m_driveSub);
}

void SetXCmd::Initialize()
{

}

void SetXCmd::Execute()
{
  m_driveSub->SetX();
}

void SetXCmd::End(bool interupted)
{

}

bool SetXCmd::IsFinished()
{
  return false;
}