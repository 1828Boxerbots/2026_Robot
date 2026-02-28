#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/sim/SparkAbsoluteEncoderSim.h>

class ArmSub : public frc2::SubsystemBase {
 public:
  ArmSub();
  ~ArmSub();

  void Periodic() override;

  void SimulationPeriodic() override;

  double GetPos1();
  double GetPos2();

  void SetPos(double pos);

 private:
  rev::spark::SparkMax m_leftarmMotor{ArmConstants::kArmMotorAPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_rightarmMotor{ArmConstants::kArmMotorBPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkAbsoluteEncoder m_leftabsArmEncoder = m_leftarmMotor.GetAbsoluteEncoder();
  rev::spark::SparkAbsoluteEncoder m_rightabsArmEncoder = m_rightarmMotor.GetAbsoluteEncoder();
  rev::spark::SparkClosedLoopController m_leftarmPid = m_leftarmMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_rightarmPid = m_rightarmMotor.GetClosedLoopController();

  double m_rotationFactor;

  // Simulation
  frc::DCMotor m_gearbox = frc::DCMotor::NEO(1);
  rev::spark::SparkMaxSim m_leftarmMotorSim{&m_leftarmMotor, &m_gearbox};
  rev::spark::SparkMaxSim m_rightarmMotorSim{&m_rightarmMotor, &m_gearbox};

  rev::spark::SparkAbsoluteEncoderSim m_leftabsArmEncoderSim = m_leftarmMotorSim.GetAbsoluteEncoderSim();
  rev::spark::SparkAbsoluteEncoderSim m_rightabsArmEncoderSim = m_rightarmMotorSim.GetAbsoluteEncoderSim();
};