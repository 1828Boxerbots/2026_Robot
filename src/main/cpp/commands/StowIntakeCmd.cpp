
#include "commands/StowIntakeCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

StowIntakeCmd::StowIntakeCmd(ArmSub *subsystem)
{
  m_subsystem = subsystem;
  AddRequirements(m_subsystem);
}

void StowIntakeCmd::Initialize()
{
  m_isFinished = false;
}

void StowIntakeCmd::Execute()
{
    
}

void StowIntakeCmd::End(bool interupted)
{

}

bool StowIntakeCmd::IsFinished()
{
  return m_isFinished;
}