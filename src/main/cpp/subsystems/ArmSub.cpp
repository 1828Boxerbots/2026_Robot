
#include "subsystems/ArmSub.h"

ArmSub::ArmSub()
{
    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig armConfig{};
    armConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);

}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    
}

double ArmSub::GetPos()
{
    return m_absArmEncoder.GetPosition();
}

void ArmSub::SetPos(float pos)
{
    m_armPid.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
}