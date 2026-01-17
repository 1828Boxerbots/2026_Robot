// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/StartEndCommand.h>
#include "Constants.h"
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>

class ShooterSub : public frc2::SubsystemBase {
 public:
  ShooterSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 rev::spark::SparkMax m_shooterMotorA{ShooterConstants::kShooter_Motor_A_ID, rev::spark::SparkMax::MotorType::kBrushless};
 rev::spark::SparkMax m_shooterMotorB{ShooterConstants::kShooter_Motor_B_ID, rev::spark::SparkMax::MotorType::kBrushless};
 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
