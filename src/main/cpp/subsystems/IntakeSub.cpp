// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSub.h"

IntakeSub::IntakeSub() = default;

// This method will be called once per scheduler run
void IntakeSub::Periodic() 
{
    frc::SmartDashboard::PutNumber("IntakeMotorA Power", m_IntakeMotorA.Get());
    frc::SmartDashboard::PutNumber("IntakeMotorB Power", m_IntakeMotorB.Get());
}
