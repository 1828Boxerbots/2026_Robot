// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/HopperSub.h"

HopperSub::HopperSub() = default;

// This method will be called once per scheduler run
void HopperSub::Periodic() 
{
     frc::SmartDashboard::PutNumber("HopperMotorA Power", m_HopperMotorA.Get());
     frc::SmartDashboard::PutNumber("HopperMotorB Power", m_HopperMotorB.Get());
}
