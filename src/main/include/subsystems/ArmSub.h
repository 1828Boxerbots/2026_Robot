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
#include <frc/simulation/SingleJointedArmSim.h>

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
  rev::spark::SparkMax m_armMotor1{ArmConstants::kArmMotorAPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_armMotor2{ArmConstants::kArmMotorBPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkAbsoluteEncoder m_absArmEncoder1 = m_armMotor1.GetAbsoluteEncoder();
  rev::spark::SparkAbsoluteEncoder m_absArmEncoder2 = m_armMotor2.GetAbsoluteEncoder();

  rev::spark::SparkClosedLoopController m_armPid1 = m_armMotor1.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_armPid2 = m_armMotor2.GetClosedLoopController();

  // Simulation
  frc::DCMotor m_gearbox = frc::DCMotor::NEO(1);
  rev::spark::SparkMaxSim m_armMotorSim1{&m_armMotor1, &m_gearbox};
  rev::spark::SparkMaxSim m_armMotorSim2{&m_armMotor2, &m_gearbox};

  rev::spark::SparkAbsoluteEncoderSim m_absArmEncoderSim1 = m_armMotorSim1.GetAbsoluteEncoderSim();
  rev::spark::SparkAbsoluteEncoderSim m_absArmEncoderSim2 = m_armMotorSim2.GetAbsoluteEncoderSim();
};