
#include "commands/ArmCmd.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

ArmCmd::ArmCmd(ArmSub *armSubsystem, IntakeSub *intakeSubsystem, double pos, double intakePower)
{
  m_armSubsystem = armSubsystem;
  m_intakeSubsystem = intakeSubsystem;

  AddRequirements(m_armSubsystem);
  AddRequirements(m_intakeSubsystem);
  
  m_targetPosition = pos;
  m_intakePower = intakePower;
}

void ArmCmd::Initialize()
{
  m_armSubsystem->SetPos(m_targetPosition);

  m_isFinished = false;
}

void ArmCmd::Execute()
{
  m_intakeSubsystem->SetPower(m_intakePower);

  m_isFinished = (CompareDoubles(m_armSubsystem->GetPos1(), m_targetPosition, ArmConstants::kPositionTolerance)
    && CompareDoubles(m_armSubsystem->GetPos2(), m_targetPosition, ArmConstants::kPositionTolerance));
}

void ArmCmd::End(bool interupted)
{
  // m_subsystem.
}

bool ArmCmd::IsFinished()
{
  return m_isFinished;
}