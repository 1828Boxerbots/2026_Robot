
#include "subsystems/ArmSub.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmSub::ArmSub()
{
    m_rotationFactor = 360;

    rev::spark::SparkMaxConfig armConfig1{};
    armConfig1.absoluteEncoder
        .Inverted(true)
        .PositionConversionFactor(m_rotationFactor);
    armConfig1.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1);

    rev::spark::SparkMaxConfig armConfig2{};
    armConfig2.absoluteEncoder
        .Inverted(false)
        .PositionConversionFactor(m_rotationFactor);
    armConfig2.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1);

    m_leftarmMotor.Configure(armConfig1,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    m_rightarmMotor.Configure(armConfig2,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_leftarmMotor.SetInverted(false);
    m_rightarmMotor.SetInverted(true);
}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Arm Motor 1 Power", m_leftarmMotor.Get());
    frc::SmartDashboard::PutNumber("Arm Motor 2 Power", m_rightarmMotor.Get());
    frc::SmartDashboard::PutNumber("Arm Encoder 1 Position", m_leftabsArmEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm Encoder 2 Position", m_rightabsArmEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm PID 1 Set Position", m_leftarmPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Arm PID 2 Set Position", m_rightarmPid.GetSetpoint());
    frc::SmartDashboard::PutString("Test", "Hearbeat");
}

void ArmSub::SimulationPeriodic()
{
    
}

double ArmSub::GetPos1()
{
    return double (m_leftabsArmEncoder.GetPosition());
}

double ArmSub::GetPos2()
{
    return double (m_rightabsArmEncoder.GetPosition());
}

void ArmSub::SetPos(double pos)
{
    m_leftarmPid.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
    m_rightarmPid.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
}