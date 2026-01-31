#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/StartEndCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

class HopperSub : public frc2::SubsystemBase {
 public:
  HopperSub();
  ~HopperSub();

  void Periodic() override;

  void SetPower(float speed);

  std::pair<double, double> GetVelocity();

 private:
  rev::spark::SparkMax m_towerMotor{HopperConstants::kTowerMotorPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_hopperMotor{HopperConstants::kHopperMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_towerEncoder = m_towerMotor.GetEncoder();
  rev::spark::SparkRelativeEncoder m_hopperEncoder = m_hopperMotor.GetEncoder();
};
