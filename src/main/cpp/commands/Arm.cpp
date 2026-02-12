
#include "commands/ArmCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmCmd::ArmCmd(ArmSub *subsystem)
{
    m_subsystem = subsystem;
    AddRequirements(m_subsystem);
}

void ArmCmd::Initialize()
{
  m_isFinished = false;
}

void ArmCmd::Execute()
{
    
}

void ArmCmd::End(bool interupted)
{

}

bool ArmCmd::IsFinished()
{
  return m_isFinished;
}