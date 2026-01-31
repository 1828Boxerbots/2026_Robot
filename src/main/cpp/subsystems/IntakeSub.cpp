
#include "subsystems/IntakeSub.h"


IntakeSub::IntakeSub()
{

}

IntakeSub::~IntakeSub() {}

void IntakeSub::Periodic()
{
    
}

void IntakeSub::SetPower(float speed)
{
    m_intakeMotor.Set(speed);
}

double IntakeSub::GetVelocity()
{
    return m_intakeEncoder.GetVelocity();
}