
#include "subsystems/ShooterSub.h"


ShooterSub::ShooterSub()
{
    m_conversionFactor = ShooterConstants::kWheelDiameter *
        std::numbers::pi /
        ShooterConstants::kMotorReduction;

    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig shooterConfig1{};
    shooterConfig1.encoder
        .VelocityConversionFactor(m_conversionFactor / 60);
    shooterConfig1.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);

    rev::spark::SparkMaxConfig shooterConfig2{};
    shooterConfig2.encoder
        .VelocityConversionFactor(m_conversionFactor / 60);
    shooterConfig2.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);

    m_leftshooterMotor.Configure(shooterConfig1,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_rightshooterMotor.Configure(shooterConfig2,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    m_leftshooterMotor.SetInverted(false);
    m_rightshooterMotor.SetInverted(true);
}

ShooterSub::~ShooterSub() {}

void ShooterSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Left Shooter Motor Power", m_leftshooterMotor.Get());
    frc::SmartDashboard::PutNumber("Right Shooter Motor Power", m_rightshooterMotor.Get());
    frc::SmartDashboard::PutNumber("Left Shooter Encoder Velocity", m_leftshooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Right Shooter Encoder Velocity", m_rightshooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Left Shooter PID Set Position", m_leftshooterPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Right Shooter PID Set Position", m_rightshooterPid.GetSetpoint());
}

void ShooterSub::SetVelocity(float velocity)
{
    m_leftshooterPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
    m_rightshooterPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

std::pair<double, double> ShooterSub::GetVelocity()
{
    double leftMotorVelocity = m_leftshooterEncoder.GetVelocity();
    double rightMotorVelocity = m_rightshooterEncoder.GetVelocity();

    return std::pair<double, double>(leftMotorVelocity, rightMotorVelocity);
}