#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ArmSub.h"
#include "subsystems/IntakeSub.h"
#include "Constants.h"
#include "Util.h"

class ArmCmd : public frc2::CommandHelper<frc2::Command, ArmCmd> 
{

 public:
  ArmCmd(ArmSub *ArmSub, IntakeSub *IntakeSub, double pos);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmSub *m_armSub = nullptr; 
  IntakeSub *m_intakeSub = nullptr;

  bool m_isFinished;
  double m_targetPosition;
};