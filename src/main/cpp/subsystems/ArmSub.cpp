
#include "subsystems/ArmSub.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmSub::ArmSub()
{
    m_rotationFactor = 360;

    // PLEASE CHANGE NAMING OF CONFIGS
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

    m_leftArmMotor.Configure(armConfig1,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    m_rightArmMotor.Configure(armConfig2,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_leftArmMotor.SetInverted(false);
    m_rightArmMotor.SetInverted(true);
}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Arm Motor 1 Power", m_leftArmMotor.Get());
    frc::SmartDashboard::PutNumber("Arm Motor 2 Power", m_rightArmMotor.Get());
    frc::SmartDashboard::PutNumber("Arm Encoder 1 Position", m_leftAbsArmEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm Encoder 2 Position", m_rightAbsArmEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm PID 1 Set Position", m_leftArmPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Arm PID 2 Set Position", m_rightArmPid.GetSetpoint());
}

void ArmSub::SimulationPeriodic()
{
    
}
 // PLEASE CHANGE NAME TO LEFT AND RIGHT
double ArmSub::GetPos1()
{
    return double (m_leftAbsArmEncoder.GetPosition());
}

double ArmSub::GetPos2()
{
    return double (m_rightAbsArmEncoder.GetPosition());
}

void ArmSub::SetPos(double pos)
{
    m_leftArmPid.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
    m_rightArmPid.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
}