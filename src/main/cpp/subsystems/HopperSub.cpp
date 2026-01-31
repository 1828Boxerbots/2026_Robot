
#include "subsystems/HopperSub.h"


HopperSub::HopperSub()
{

}

HopperSub::~HopperSub() {}

void HopperSub::Periodic()
{
    
}

void HopperSub::SetPower(float speed)
{
    m_towerMotor.Set(speed);
    m_hopperMotor.Set(speed);
}

std::pair<double, double> HopperSub::GetVelocity()
{
    double towerVelocity = m_towerEncoder.GetVelocity();
    double hopperVelocity = m_towerEncoder.GetVelocity();

    return std::pair<double, double> (towerVelocity,hopperVelocity);
}