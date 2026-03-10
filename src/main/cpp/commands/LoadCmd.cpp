
#include "commands/LoadCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

LoadCmd::LoadCmd(IntakeSub *subsystem, ArmSub *armSub, double speed)
{
  m_subsystem = subsystem;
  m_armSub = armSub;
  m_speed = speed;
  AddRequirements(m_subsystem);
  AddRequirements(m_armSub);
}

void LoadCmd::Initialize()
{
  m_subsystem->SetPower(m_speed);
}

void LoadCmd::Execute()
{
  m_armSub->SetPower(m_armSpeed);
}

void LoadCmd::End(bool interupted)
{
  m_subsystem->SetPower(0);
  m_armSub->SetPower(0.0);
}

bool LoadCmd::IsFinished()
{
  return false;
}