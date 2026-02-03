
#include "subsystems/HopperSub.h"


HopperSub::HopperSub()
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

HopperSub::~HopperSub() {}

void HopperSub::Periodic()
{
    
}

void HopperSub::SetVelocity(float velocity)
{
    m_towerPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
    m_hopperPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

std::pair<double, double> HopperSub::GetVelocity()
{
    double towerVelocity = m_towerEncoder.GetVelocity();
    double hopperVelocity = m_towerEncoder.GetVelocity();

    return std::pair<double, double> (towerVelocity,hopperVelocity);
}