#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/StartEndCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();
  ~IntakeSub();

  void Periodic() override;

  void SetPower(float speed);

  double GetVelocity();

 private:
  rev::spark::SparkMax m_intakeMotor{IntakeConstants::kIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();

};