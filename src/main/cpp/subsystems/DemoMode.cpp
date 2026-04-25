#include "subsystems/DemoMode.h"
#include <frc/smartdashboard/SmartDashboard.h>

bool DemoMode::s_DemoModeEnabled = false;

DemoMode::DemoMode()
{
    SetName("DemoMode");
}

DemoMode::~DemoMode()
{
    // NOTE: Currently does nothing
}

void DemoMode::Init()
{
    // NOTE: Currently does nothing
}

void DemoMode::Periodic()
{
    s_DemoModeEnabled = m_DemoSwitch.Get();

    frc::SmartDashboard::PutBoolean("Demo Mode Enabled", s_DemoModeEnabled);
}