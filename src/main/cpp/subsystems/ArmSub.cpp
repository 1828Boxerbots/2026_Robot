
#include "subsystems/ArmSub.h"
#include "RobotContainer.h"

#include <frc2/command/FunctionalCommand.h>

ArmSub::ArmSub()
{

}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    
}

double ArmSub::GetPos()
{
    return m_absArmEncoder.GetPosition();
}