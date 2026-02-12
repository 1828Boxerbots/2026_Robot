
#include "subsystems/IntakeSub.h"


IntakeSub::IntakeSub()
{
    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig intakeConfig{};
    intakeConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);
}

IntakeSub::~IntakeSub() {}

void IntakeSub::Periodic()
{
    
}

void IntakeSub::SetVelocity(float velocity)
{
    m_intakePid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

double IntakeSub::GetVelocity()
{
    return m_intakeEncoder.GetVelocity();
}

void IntakeSub::Set(float power)
{
    m_intakeMotor.Set(power);
}