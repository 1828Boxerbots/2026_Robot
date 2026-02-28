
#include "commands/LoadCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

LoadCmd::LoadCmd(IntakeSub *subsystem, double power)
{
  m_subsystem = subsystem;
  m_power = power;
  AddRequirements(m_subsystem);
}

void LoadCmd::Initialize()
{
  m_subsystem->SetPower(m_power);
}

void LoadCmd::Execute()
{
    
}

void LoadCmd::End(bool interupted)
{
  m_subsystem->SetPower(0);
}

bool LoadCmd::IsFinished()
{
  return false;
}