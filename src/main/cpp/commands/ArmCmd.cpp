
#include "commands/ArmCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmCmd::ArmCmd(ArmSub *Armsub, IntakeSub *Intakesub, double pos)
{
  m_armSub = Armsub;
  m_intakeSub = Intakesub;
  AddRequirements(m_armSub);
  AddRequirements(m_intakeSub);
  
  m_targetPosition = pos;
}

void ArmCmd::Initialize()
{
  m_armSub->SetPos(m_targetPosition);
  m_intakeSub->SetPower(-IntakeConstants::kIntakePower);

  m_isFinished = false;
}

void ArmCmd::Execute()
{
  m_isFinished = (CompareDoubles(m_armSub->GetPos1(), m_targetPosition, ArmConstants::kPositionTolerance)
    && CompareDoubles(m_armSub->GetPos2(), m_targetPosition, ArmConstants::kPositionTolerance));
}

void ArmCmd::End(bool interupted)
{
  // m_subsystem.
  m_intakeSub->SetPower(0);
}

bool ArmCmd::IsFinished()
{
  return m_isFinished;
}