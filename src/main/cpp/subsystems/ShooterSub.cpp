
#include "subsystems/ShooterSub.h"


ShooterSub::ShooterSub()
{

}

ShooterSub::~ShooterSub() {}

void ShooterSub::Periodic()
{

}

void ShooterSub::SetPower(float speed)
{
    m_shooterMotor1.Set(speed);
    m_shooterMotor2.Set(speed);
}

std::pair<double, double> ShooterSub::GetVelocity()
{
    double motor1Velocity = m_shooterEncoder1.GetVelocity();
    double motor2Velocity = m_shooterEncoder2.GetVelocity();

    return std::pair<double, double>(motor1Velocity, motor2Velocity);
}