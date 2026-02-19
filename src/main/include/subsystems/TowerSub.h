#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>

class TowerSub : public frc2::SubsystemBase {
 public:
  TowerSub();
  ~TowerSub();

  void Periodic() override;

  void SetVelocity(float velocity);

  double GetVelocity();

 private:
  rev::spark::SparkMax m_towerMotor{TowerConstants::kTowerMotorPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkRelativeEncoder m_towerEncoder = m_towerMotor.GetEncoder();
  rev::spark::SparkClosedLoopController m_towerPid = m_towerMotor.GetClosedLoopController();

  double m_conversionFactor;
};
