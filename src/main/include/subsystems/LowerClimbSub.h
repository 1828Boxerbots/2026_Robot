// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include <rev/SparkMax.h>


class LowerClimbSub : public frc2::SubsystemBase {
 public:
  LowerClimbSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 rev::spark::SparkMax m_LowerClimbMotorA{LowerClimbConstants::kLowerClimb_Motor_A_ID, rev::spark::SparkMax::MotorType::kBrushless};
 rev::spark::SparkMax m_LowerClimbMotorB{LowerClimbConstants::kLowerClimb_Motor_B_ID, rev::spark::SparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
