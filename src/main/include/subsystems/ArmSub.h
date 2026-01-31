#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

class ArmSub : public frc2::SubsystemBase {
 public:
  ArmSub();
  ~ArmSub();

  void Periodic() override;

  double GetPos();

 private:
  rev::spark::SparkMax m_armMotor{ArmConstants::kArmMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkAbsoluteEncoder m_absArmEncoder = m_armMotor.GetAbsoluteEncoder();

};