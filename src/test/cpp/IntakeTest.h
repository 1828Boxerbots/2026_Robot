#pragma once

#include "subsystems/ArmSub.h"
#include "subsystems/HopperSub.h"
#include "subsystems/IntakeSub.h"
#include "subsystems/ShooterSub.h"
#include "Constants.h"

#include <gtest/gtest.h>
#include <rev/SparkSim.h>

class IntakeTest : public testing::Test {
 protected:
  Intake intake;

  rev::spark::SparkMax m_intakeMotor{IntakeConstants::kIntakeMotorPort, rev::spark::SparkMax::MotorType::kBrushless};
}