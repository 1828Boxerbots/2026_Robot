
#include "subsystems/ArmSub.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmSub::ArmSub()
{
    m_rotationFactor = 6.84;
    double ffStowConstant = 0.15;

    rev::spark::SparkMaxConfig armConfig{};
    armConfig.absoluteEncoder
        .Inverted(true)
        .PositionConversionFactor(m_rotationFactor);
    armConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(0, 0, 0)
        .OutputRange(-1, 0.15)
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, m_rotationFactor)
        .VelocityFF(2);


    m_leftArmMotor.Configure(armConfig,
    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_rightArmMotor.Configure(armConfig,
    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

    // double ffStowConstant = 0.15;
    // double ffDeployContant = 0.05;

    // rev::spark::SparkMaxConfig stowLeftArmConfig{};
    // stowLeftArmConfig.absoluteEncoder
    //     .Inverted(true)
    //     .PositionConversionFactor(m_rotationFactor);
    // stowLeftArmConfig.closedLoop
    //     .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
    //     .Pid(0, 0, 0)
    //     .OutputRange(-1, 1)
    //     .VelocityFF(ffStowConstant);

    // rev::spark::SparkMaxConfig stowRightArmConfig{};
    // stowRightArmConfig.absoluteEncoder
    //     .Inverted(true)
    //     .PositionConversionFactor(m_rotationFactor);
    // stowRightArmConfig.closedLoop
    //     .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
    //     .Pid(0, 0, 0)
    //     .OutputRange(-1, 1)
    //     .VelocityFF(ffStowConstant);

    // rev::spark::SparkMaxConfig deployLeftArmConfig{};
    // deployLeftArmConfig.absoluteEncoder
    //     .Inverted(false)
    //     .PositionConversionFactor(m_rotationFactor);
    // deployLeftArmConfig.closedLoop
    //     .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
    //     .Pid(0, 0, 0)
    //     .OutputRange(-1, 1)
    //     .VelocityFF(ffDeployContant);

    // rev::spark::SparkMaxConfig deployRightArmConfig{};
    // deployRightArmConfig.absoluteEncoder
    //     .Inverted(false)
    //     .PositionConversionFactor(m_rotationFactor);
    // deployRightArmConfig.closedLoop
    //     .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
    //     .Pid(0, 0, 0)
    //     .OutputRange(-1, 1)
    //     .VelocityFF(ffDeployContant);


    // m_leftArmMotor.Configure(stowLeftArmConfig,
    //     rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    //     rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    // m_rightArmMotor.Configure(stowRightArmConfig,
    //     rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    //     rev::spark::SparkBase::PersistMode::kPersistParameters);

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
    if(pos == ArmConstants::kStowedPosition)
    {
        // std::cout << "Stow" << std::endl;

        m_leftArmPid.SetReference(
            pos, 
            rev::spark::SparkMax::ControlType::kPosition
        );
        m_rightArmPid.SetReference(
            pos, 
            rev::spark::SparkMax::ControlType::kPosition
        );
    }
    else
    {
        // std::cout << "Deploy" << std::endl;

        m_leftArmPid.SetReference(
            pos, 
            rev::spark::SparkMax::ControlType::kPosition
        );
        m_rightArmPid.SetReference(
            pos, 
            rev::spark::SparkMax::ControlType::kPosition
        );
    }
}

void ArmSub::SetPower(float power)
{
    m_leftArmMotor.Set(power);
    m_rightArmMotor.Set(power);
}