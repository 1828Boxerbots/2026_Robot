
#include "subsystems/ArmSub.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmSub::ArmSub()
{
    // frc::SmartDashboard::PutNumber("simPositionInput", 0.0);

    rev::spark::SparkMaxConfig armConfig{};
    armConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1);

    m_armMotor1.SetInverted(false);
    m_armMotor2.SetInverted(true);
}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    
}

void ArmSub::SimulationPeriodic()
{
    // double position = frc::SmartDashboard::GetNumber("simPositionInput", 0.0);
    // frc::SmartDashboard::PutNumber("simCheck", position);
    // m_armMotorSim1.SetPosition(position); 
    // frc::SmartDashboard::PutNumber("simPositonOutput", m_armMotorSim1.GetPosition());
}

double ArmSub::GetPos1()
{
    return double (m_absArmEncoder1.GetPosition());
}

double ArmSub::GetPos2()
{
    return double (m_absArmEncoder2.GetPosition());
}

void ArmSub::SetPos(double pos)
{
    m_armPid1.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
    m_armPid2.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
}