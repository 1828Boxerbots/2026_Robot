
#include "subsystems/ArmSub.h"

ArmSub::ArmSub()
{
    rev::spark::SparkMaxConfig armConfig{};
    armConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1);

    m_armMotor1.SetInverted(false);
    m_armMotor2.SetInverted(true);
}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    
}

double ArmSub::GetPos1()
{
    return double (m_absArmEncoder1.GetPosition());
}

double ArmSub::GetPos2()
{
    return double (m_absArmEncoder2.GetPosition());
}

void ArmSub::SetPos(double pos)
{
    m_armPid1.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
    m_armPid2.SetReference(pos, rev::spark::SparkMax::ControlType::kPosition);
}