#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ArmSub.h"

class StowIntakeCmd : public frc2::CommandHelper<frc2::Command, StowIntakeCmd> 
{
  
 public:
  StowIntakeCmd(ArmSub *m_subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmSub *m_subsystem = nullptr; 

  bool m_isFinished = false;
};