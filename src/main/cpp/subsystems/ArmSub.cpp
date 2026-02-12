
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

    m_armMotor1.SetInverted(false);
    m_armMotor2.SetInverted(true);
}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    
}

std::pair<double, double> ArmSub::GetPos()
{
    double encoderValue1 = m_absArmEncoder1.GetPosition();
    double encoderValue2 = m_absArmEncoder2.GetPosition();
    return std::pair<double, double> (encoderValue1, encoderValue2);
}

void ArmSub::SetPos(float pos)
{
    m_armPid1.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
    m_armPid2.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
}