#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "Util.h"

class SetXCmd : public frc2::CommandHelper<frc2::Command, SetXCmd> 
{

 public:
  SetXCmd(DriveSubsystem *m_driveSub);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem *m_driveSub = nullptr; 
};