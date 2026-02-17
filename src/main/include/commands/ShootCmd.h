#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSub.h"
#include "subsystems/TowerSub.h"

class ShootCmd : public frc2::CommandHelper<frc2::Command, ShootCmd> 
{
 public:
  ShootCmd(ShooterSub *shootSubsystem, TowerSub *towerSubsystem, double shootSpeed,  double towerSpeed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSub *m_shootSubsystem = nullptr; 
  TowerSub *m_towerSubsystem = nullptr; 
  double m_shootSpeed = 0;
  double m_towerSpeed = 0;
  bool m_isFinished = false;
};