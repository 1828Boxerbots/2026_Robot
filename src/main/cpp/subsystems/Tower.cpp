
#include "subsystems/TowerSub.h"


TowerSub::TowerSub()
{
    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig hopperConfig{};
    hopperConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);
}

TowerSub::~TowerSub() {}

void TowerSub::Periodic()
{
    
}

void TowerSub::SetVelocity(float velocity)
{
    m_towerPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

double TowerSub::GetVelocity()
{
    double towerVelocity = m_towerEncoder.GetVelocity();

    return towerVelocity;
}