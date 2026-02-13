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
  rev::spark::SparkMax m_shooterMotor1{ShooterConstants::kShooterMotorAPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_shooterMotor2{ShooterConstants::kShooterMotorBPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_shooterEncoder1 = m_shooterMotor1.GetEncoder();
  rev::spark::SparkRelativeEncoder m_shooterEncoder2 = m_shooterMotor2.GetEncoder();

  rev::spark::SparkClosedLoopController m_shooterPid1 = m_shooterMotor1.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_shooterPid2 = m_shooterMotor2.GetClosedLoopController();
};