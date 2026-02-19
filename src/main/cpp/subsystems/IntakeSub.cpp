
#include "subsystems/IntakeSub.h"


IntakeSub::IntakeSub()
{
    m_conversionFactor = IntakeConstants::kWheelDiameter *
        std::numbers::pi /
        IntakeConstants::kMotorReduction;

    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig intakeConfig{};
    intakeConfig.encoder
        .VelocityConversionFactor(m_conversionFactor / 60);
    intakeConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);

    m_intakeMotor.Configure(intakeConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
}

IntakeSub::~IntakeSub() {}

void IntakeSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Intake Motor Power", m_intakeMotor.Get());
    frc::SmartDashboard::PutNumber("Intake Enocder Velocity", m_intakeEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Intake PID Set Position", m_intakePid.GetSetpoint());
}

void IntakeSub::SetVelocity(float velocity)
{
    m_intakePid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

double IntakeSub::GetVelocity()
{
    return m_intakeEncoder.GetVelocity();
}

void IntakeSub::Set(float power)
{
    m_intakeMotor.Set(power);
}