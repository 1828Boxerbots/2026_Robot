// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LowerClimbSub.h"

LowerClimbSub::LowerClimbSub() = default;

// This method will be called once per scheduler run
void LowerClimbSub::Periodic() 
{
     frc::SmartDashboard::PutNumber("LowerClimbMotorA Power", m_LowerClimbMotorA.Get());
     frc::SmartDashboard::PutNumber("LowerClimbMotorB Power", m_LowerClimbMotorB.Get());
}


