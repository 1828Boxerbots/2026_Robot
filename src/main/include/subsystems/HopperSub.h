// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
class HopperSub : public frc2::SubsystemBase {
 public:
  HopperSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 rev::spark::SparkMax m_HopperMotorA{HopperConstants::kHopper_Motor_A_ID, rev::spark::SparkMax::MotorType::kBrushless};
 rev::spark::SparkMax m_HopperMotorB{HopperConstants::kHopper_Motor_B_ID, rev::spark::SparkMax::MotorType::kBrushless};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
