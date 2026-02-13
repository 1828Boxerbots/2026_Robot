
#include "commands/LoadCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

LoadCmd::LoadCmd(IntakeSub *subsystem)
{
  m_subsystem = subsystem;
  AddRequirements(m_subsystem);
}

void LoadCmd::Initialize()
{
  m_isFinished = false;
}

void LoadCmd::Execute()
{
    
}

void LoadCmd::End(bool interupted)
{

}

bool LoadCmd::IsFinished()
{
  return m_isFinished;
}