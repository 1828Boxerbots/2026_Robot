#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class DemoMode : public frc2::SubsystemBase
    {
    public:
        DemoMode();
        ~DemoMode();

        void Init();
        void Periodic() override;

        static bool GetDemoMode() { return s_DemoModeEnabled; }

    protected:
    private:
    frc::DigitalInput m_DemoSwitch {DemoModeConstants::kDemoSwitchPort};
        static bool s_DemoModeEnabled;
    };