
#include "commands/DeployIntakeCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

DeployIntakeCmd::DeployIntakeCmd(ArmSub *subsystem)
{
    m_subsystem = subsystem;
    AddRequirements(m_subsystem);
}

void DeployIntakeCmd::Initialize()
{
  m_isFinished = false;
}

void DeployIntakeCmd::Execute()
{
    
}

void DeployIntakeCmd::End(bool interupted)
{

}

bool DeployIntakeCmd::IsFinished()
{
  return m_isFinished;
}