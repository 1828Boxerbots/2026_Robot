#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>

class ArmSub : public frc2::SubsystemBase {
 public:
  ArmSub();
  ~ArmSub();

  void Periodic() override;

  std::pair<double, double> GetPos();

  void SetPos(float pos);

 private:
  rev::spark::SparkMax m_armMotor1{ArmConstants::kArmMotorAPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_armMotor2{ArmConstants::kArmMotorBPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkAbsoluteEncoder m_absArmEncoder1 = m_armMotor1.GetAbsoluteEncoder();
  rev::spark::SparkAbsoluteEncoder m_absArmEncoder2 = m_armMotor2.GetAbsoluteEncoder();

  rev::spark::SparkClosedLoopController m_armPid1 = m_armMotor1.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_armPid2 = m_armMotor2.GetClosedLoopController();
};