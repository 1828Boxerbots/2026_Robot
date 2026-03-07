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

  void SetPower(float power);

 private:
  rev::spark::SparkMax m_leftArmMotor{ArmConstants::kLeftArmMotorPort, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_rightArmMotor{ArmConstants::kRightArmMotorPort, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkAbsoluteEncoder m_leftAbsArmEncoder = m_leftArmMotor.GetAbsoluteEncoder();
  rev::spark::SparkAbsoluteEncoder m_rightAbsArmEncoder = m_rightArmMotor.GetAbsoluteEncoder();

  rev::spark::SparkClosedLoopController m_leftArmPid = m_leftArmMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_rightArmPid = m_rightArmMotor.GetClosedLoopController();

  double m_rotationFactor;

  // // Simulation
  // frc::DCMotor m_gearbox = frc::DCMotor::NEO(1);
  // rev::spark::SparkMaxSim m_armMotorSim1{&m_armMotor1, &m_gearbox};
  // rev::spark::SparkMaxSim m_armMotorSim2{&m_armMotor2, &m_gearbox};

  // rev::spark::SparkAbsoluteEncoderSim m_absArmEncoderSim1 = m_armMotorSim1.GetAbsoluteEncoderSim();
  // rev::spark::SparkAbsoluteEncoderSim m_absArmEncoderSim2 = m_armMotorSim2.GetAbsoluteEncoderSim();
};