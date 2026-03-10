#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSub.h"
#include "subsystems/ArmSub.h"

class LoadCmd : public frc2::CommandHelper<frc2::Command, LoadCmd> 
{

 public:
  LoadCmd(IntakeSub *m_subsystem, ArmSub *m_armSub, double speed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSub *m_subsystem = nullptr; 
  ArmSub *m_armSub = nullptr; 
  double m_speed = 0;
  double m_armSpeed = 0.05;
  bool m_isFinished = false;
};