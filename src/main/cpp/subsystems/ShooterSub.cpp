
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

    m_shooterMotor1.Configure(shooterConfig1,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_shooterMotor2.Configure(shooterConfig2,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
    
    m_shooterMotor1.SetInverted(false);
    m_shooterMotor2.SetInverted(true);
}

ShooterSub::~ShooterSub() {}

void ShooterSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Shooter Motor 1 Power", m_shooterMotor1.Get());
    frc::SmartDashboard::PutNumber("Shooter Motor 2 Power", m_shooterMotor2.Get());
    frc::SmartDashboard::PutNumber("Shooter Encoder 1 Velocity", m_shooterEncoder1.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Encoder 2 Velocity", m_shooterEncoder2.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter PID 1 Set Position", m_shooterPid1.GetSetpoint());
    frc::SmartDashboard::PutNumber("Shooter PID 2 Set Position", m_shooterPid2.GetSetpoint());
}

void ShooterSub::SetVelocity(float velocity)
{
    m_shooterPid1.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
    m_shooterPid2.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

double ShooterSub::GetVelocity1()
{
    return m_shooterEncoder1.GetVelocity();
}

double ShooterSub::GetVelocity2()
{
    return m_shooterEncoder2.GetVelocity();
}