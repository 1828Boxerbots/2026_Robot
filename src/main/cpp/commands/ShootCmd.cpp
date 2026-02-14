
#include "commands/ShootCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ShootCmd::ShootCmd(ShooterSub *subsystem, double speed)
{
    m_subsystem = subsystem;
    m_speed = speed;
    AddRequirements(m_subsystem);
}

void ShootCmd::Initialize()
{
  m_subsystem->SetVelocity(m_speed);
}

void ShootCmd::Execute()
{
    
}

void ShootCmd::End(bool interupted)
{

}

bool ShootCmd::IsFinished()
{
    return false;
}