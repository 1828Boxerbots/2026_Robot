
#include "commands/LoadCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

LoadCmd::LoadCmd(IntakeSub *subsystem, double speed)
{
  m_subsystem = subsystem;
  m_speed = speed;
  AddRequirements(m_subsystem);
}

void LoadCmd::Initialize()
{
  m_subsystem->SetVelocity(m_speed);
}

void LoadCmd::Execute()
{
    
}

void LoadCmd::End(bool interupted)
{
  m_subsystem->SetVelocity(0);
}

bool LoadCmd::IsFinished()
{
  return false;
}