#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSub.h"

class LoadCmd : public frc2::CommandHelper<frc2::Command, LoadCmd> 
{

 public:
  LoadCmd(IntakeSub *m_subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSub *m_subsystem = nullptr; 

  bool m_isFinished = false;
};