#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSub.h"
#include "subsystems/TowerSub.h"
#include "subsystems/VisionSub.h"

class ShootCmd : public frc2::CommandHelper<frc2::Command, ShootCmd> 
{
 public:
  ShootCmd(ShooterSub *shootSubsystem, TowerSub *towerSubsystem, double shootVelocity,  double towerPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSub *m_shootSubsystem = nullptr; 
  TowerSub *m_towerSubsystem = nullptr; 
  double m_shootVelocity = 0;
  double m_towerPower = 0;
  bool m_isFinished = false;

  double m_numeratorCalculation;
  double m_heightCalculation;
  double m_angleCalculation;
};