#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();
  ~ShooterSub();

  void Periodic() override;

  void SetVelocity(float velocity);

  std::pair<double, double> GetVelocity();

 private:
  rev::spark::SparkMax m_leftshooterMotor{ShooterConstants::kLeftShooterMotorPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_rightshooterMotor{ShooterConstants::kRightShooterMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_leftshooterEncoder = m_leftshooterMotor.GetEncoder();
  rev::spark::SparkRelativeEncoder m_rightshooterEncoder = m_rightshooterMotor.GetEncoder();
  rev::spark::SparkClosedLoopController m_leftshooterPid = m_leftshooterMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_rightshooterPid = m_rightshooterMotor.GetClosedLoopController();

  double m_conversionFactor;
};