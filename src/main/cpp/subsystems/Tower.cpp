
#include "subsystems/TowerSub.h"


TowerSub::TowerSub()
{
    m_conversionFactor = TowerConstants::kWheelDiameter *
        std::numbers::pi /
        TowerConstants::kMotorReduction;

    double velocityFeedForward =
        1 / OtherConstants::kNeo2FeedForwardRps;

    rev::spark::SparkMaxConfig towerConfig{};
    towerConfig.encoder
        .VelocityConversionFactor(m_conversionFactor / 60);
    towerConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        .VelocityFF(velocityFeedForward);

    m_towerMotor.Configure(towerConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);
}

TowerSub::~TowerSub() {}

void TowerSub::Periodic()
{
    frc::SmartDashboard::PutNumber("Tower Motor Power", m_towerMotor.Get());
    frc::SmartDashboard::PutNumber("Tower Enocder Velocity", m_towerEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Tower PID Set Position", m_towerPid.GetSetpoint());
}

void TowerSub::SetVelocity(float velocity)
{
    m_towerPid.SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}

double TowerSub::GetVelocity()
{
    double towerVelocity = m_towerEncoder.GetVelocity();

    return towerVelocity;
}