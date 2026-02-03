#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>

class IntakeSub : public frc2::SubsystemBase {
 public:
  IntakeSub();
  ~IntakeSub();

  void Periodic() override;

  void SetVelocity(float velocity);

  double GetVelocity();

 private:
  rev::spark::SparkMax m_intakeMotor{IntakeConstants::kIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();

  rev::spark::SparkClosedLoopController m_intakePid = m_intakeMotor.GetClosedLoopController();
};