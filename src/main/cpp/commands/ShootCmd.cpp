
#include "commands/ShootCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ShootCmd::ShootCmd(ShooterSub *subsystem)
{
    m_subsystem = subsystem;
    AddRequirements(m_subsystem);
}

void ShootCmd::Initialize()
{
  m_isFinished = false;
}

void ShootCmd::Execute()
{
    
}

void ShootCmd::End(bool interupted)
{

}

bool ShootCmd::IsFinished()
{
    return m_isFinished;
}