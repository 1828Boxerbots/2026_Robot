#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ShooterSub.h"

class ShootCmd : public frc2::CommandHelper<frc2::Command, ShootCmd> 
{
 public:
  ShootCmd(ShooterSub *m_subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSub *m_subsystem = nullptr; 

  bool m_isFinished = false;
};