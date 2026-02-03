
#include "subsystems/ShooterSub.h"


ShooterSub::ShooterSub()
{
    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig shooterConfig{};
    shooterConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);

    m_shooterMotor1.SetInverted(false);
    m_shooterMotor2.SetInverted(true);
}

ShooterSub::~ShooterSub() {}

void ShooterSub::Periodic()
{

}

void ShooterSub::SetVelocity(float velocity)
{
    m_shooterPid1.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
    m_shooterPid2.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

std::pair<double, double> ShooterSub::GetVelocity()
{
    double motor1Velocity = m_shooterEncoder1.GetVelocity();
    double motor2Velocity = m_shooterEncoder2.GetVelocity();

    return std::pair<double, double>(motor1Velocity, motor2Velocity);
}