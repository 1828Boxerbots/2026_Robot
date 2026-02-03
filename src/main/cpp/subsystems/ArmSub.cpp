
#include "subsystems/ArmSub.h"
#include "RobotContainer.h"

#include <frc2/command/FunctionalCommand.h>

ArmSub::ArmSub()
{
    SparkMaxConfig armConfig{};
    armConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(false)
        .PositionWrappingInputRange(ArmConstants::kStowedPosition, ArmConstants::kDeloyedPositon);


}

ArmSub::~ArmSub() {}

void ArmSub::Periodic()
{
    
}

double ArmSub::GetPos()
{
    return m_absArmEncoder.GetPosition();
}