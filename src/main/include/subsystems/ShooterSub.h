#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/RelativeEncoder.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();
  ~ShooterSub();

  void Periodic() override;

  void SetVelocity(float velocity);

  void SetPower(float power);

  double GetTargetVelocity();

  std::pair<double, double> GetVelocity();

  double GetLeftVelocity();

 private:
  rev::spark::SparkMax m_leftShooterMotor{ShooterConstants::kLeftShooterMotorPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_rightShooterMotor{ShooterConstants::kRightShooterMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_leftShooterEncoder = m_leftShooterMotor.GetEncoder();
  rev::spark::SparkRelativeEncoder m_rightShooterEncoder = m_rightShooterMotor.GetEncoder();

  rev::spark::SparkClosedLoopController m_leftShooterPid = m_leftShooterMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_rightShooterPid = m_rightShooterMotor.GetClosedLoopController();

  double m_conversionFactor;

  double distanceVelocity;
  std::shared_ptr<nt::NetworkTable> redTable;
  nt::DoubleArraySubscriber redSub;
  std::shared_ptr<nt::NetworkTable> blueTable;
  nt::DoubleArraySubscriber blueSub;
};