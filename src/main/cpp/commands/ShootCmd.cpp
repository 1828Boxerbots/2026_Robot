
#include "commands/ShootCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

  ShootCmd::ShootCmd(ShooterSub *shootSubsystem, TowerSub *towerSubsystem, double shootSpeed,  double towerSpeed)
{
  m_shootSubsystem = shootSubsystem;
  m_towerSubsystem = towerSubsystem;
  m_shootSpeed = shootSpeed;
  m_towerSpeed = towerSpeed;
  AddRequirements(m_shootSubsystem);
  AddRequirements(m_towerSubsystem);
}

void ShootCmd::Initialize()
{
  m_shootSubsystem->SetVelocity(m_shootSpeed);
  m_towerSubsystem->SetVelocity(m_towerSpeed);
}

void ShootCmd::Execute()
{
    
}

void ShootCmd::End(bool interupted)
{
  m_shootSubsystem->SetVelocity(0);
  m_towerSubsystem->SetVelocity(0);
}

bool ShootCmd::IsFinished()
{
    return false;
}