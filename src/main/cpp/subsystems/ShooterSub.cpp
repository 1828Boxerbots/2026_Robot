
#include "subsystems/ShooterSub.h"


ShooterSub::ShooterSub()
{
    // PLEASE CHANGE NAME OF CONFIGS
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

    m_leftShooterMotor.Configure(shooterConfig1,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_rightShooterMotor.Configure(shooterConfig2,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    m_leftShooterMotor.SetInverted(false);
    m_rightShooterMotor.SetInverted(true);
}

ShooterSub::~ShooterSub() {}

void ShooterSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Shooter Motor 1 Power", m_leftShooterMotor.Get());
    frc::SmartDashboard::PutNumber("Shooter Motor 2 Power", m_rightShooterMotor.Get());
    frc::SmartDashboard::PutNumber("Shooter Encoder 1 Velocity", m_leftShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Encoder 2 Velocity", m_rightShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter PID 1 Set Position", m_leftShooterPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Shooter PID 2 Set Position", m_rightShooterPid.GetSetpoint());
}

void ShooterSub::SetVelocity(float velocity)
{
    m_leftShooterPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
    m_rightShooterPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

std::pair<double, double> ShooterSub::GetVelocity()
{
    double motor1Velocity = m_leftShooterEncoder.GetVelocity();
    double motor2Velocity = m_rightShooterEncoder.GetVelocity();

    return std::pair<double, double>(motor1Velocity, motor2Velocity);
}