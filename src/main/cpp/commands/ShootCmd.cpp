
#include "commands/ShootCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <math.h>

  ShootCmd::ShootCmd(ShooterSub *shootSubsystem, TowerSub *towerSubsystem, double shootVelocity,  double towerPower)
{
  m_shootSubsystem = shootSubsystem;
  m_towerSubsystem = towerSubsystem;
  m_shootVelocity = shootVelocity;
  m_towerPower = towerPower;
  AddRequirements(m_shootSubsystem);
  AddRequirements(m_towerSubsystem);
}

void ShootCmd::Initialize()
{
  m_shootSubsystem->SetVelocity(m_shootVelocity);
}

void ShootCmd::Execute()
{
  if((m_shootSubsystem->GetVelocity1() == m_shootVelocity) && (m_shootSubsystem->GetVelocity2() == m_shootVelocity))
  {
    m_towerSubsystem->SetPower(m_towerPower);
  }
}

void ShootCmd::End(bool interupted)
{
  m_shootSubsystem->SetVelocity(0);
  m_towerSubsystem->SetPower(0);
}

bool ShootCmd::IsFinished()
{
    return false;
}