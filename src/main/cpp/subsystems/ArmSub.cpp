
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

    m_armMotor1.Configure(armConfig1,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    m_armMotor2.Configure(armConfig2,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_armMotor1.SetInverted(false);
    m_armMotor2.SetInverted(true);
}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Arm Motor 1 Power", m_armMotor1.Get());
    frc::SmartDashboard::PutNumber("Arm Motor 2 Power", m_armMotor2.Get());
    frc::SmartDashboard::PutNumber("Arm Encoder 1 Position", m_absArmEncoder1.GetPosition());
    frc::SmartDashboard::PutNumber("Arm Encoder 2 Position", m_absArmEncoder2.GetPosition());
    frc::SmartDashboard::PutNumber("Arm PID 1 Set Position", m_armPid1.GetSetpoint());
    frc::SmartDashboard::PutNumber("Arm PID 2 Set Position", m_armPid2.GetSetpoint());
}

void ArmSub::SimulationPeriodic()
{
    
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