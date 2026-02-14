
#include "commands/ArmCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmCmd::ArmCmd(ArmSub *subsystem, double pos)
{
  m_subsystem = subsystem;
  AddRequirements(m_subsystem);
  
  m_targetPosition = pos;
}

void ArmCmd::Initialize()
{
  m_subsystem->SetPos(m_targetPosition);

  m_isFinished = false;
}

void ArmCmd::Execute()
{
  m_isFinished = (CompareDoubles(m_subsystem->GetPos1(), m_targetPosition, ArmConstants::kPositionTolerance)
    && CompareDoubles(m_subsystem->GetPos2(), m_targetPosition, ArmConstants::kPositionTolerance));
}

void ArmCmd::End(bool interupted)
{
  // m_subsystem.
}

bool ArmCmd::IsFinished()
{
  return m_isFinished;
}