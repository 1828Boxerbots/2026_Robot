#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ArmSub.h"

class DeployIntakeCmd : public frc2::CommandHelper<frc2::Command, DeployIntakeCmd> 
{

 public:
  DeployIntakeCmd(ArmSub *m_subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmSub *m_subsystem = nullptr; 

  bool m_isFinished = false;
};