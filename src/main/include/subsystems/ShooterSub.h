#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/StartEndCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();
  ~ShooterSub();

  void Periodic() override;

  void SetPower(float speed);

  std::pair<double, double> GetVelocity();

 private:
  rev::spark::SparkMax m_shooterMotor1{ShooterConstants::kShooterMotorAPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_shooterMotor2{ShooterConstants::kShooterMotorBPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_shooterEncoder1 = m_shooterMotor1.GetEncoder();
  rev::spark::SparkRelativeEncoder m_shooterEncoder2 = m_shooterMotor2.GetEncoder();

};