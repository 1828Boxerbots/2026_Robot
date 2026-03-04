
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

  m_tolerance = 0.001;
}

void ShootCmd::Initialize()
{
  m_shootSubsystem->SetVelocity(m_shootSpeed);
}

void ShootCmd::Execute()
{
  if ((m_shootSubsystem->GetLeftVelocity() < (m_shootSpeed + m_tolerance)) && (m_shootSubsystem->GetLeftVelocity() > (m_shootSpeed - m_tolerance)))
  {
    m_towerSubsystem->SetPower(m_towerSpeed);
  }
  
    std::cout << m_shootSpeed << std::endl;
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