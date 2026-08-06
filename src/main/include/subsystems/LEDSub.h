#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>

#include "Constants.h"


class LEDSub : public frc2::SubsystemBase {
 public:
  LEDSub();

  void Init();
  
  void Periodic() override;

    void SetPixelRGB(unsigned int i, uint8_t r, uint8_t g, uint8_t b);
    void SetRangeRGB(unsigned int start, unsigned int end, uint8_t r, uint8_t g, uint8_t b);

    void ApplyBuffer();

 private:

    frc::AddressableLED m_AddressableLED {LEDConstants::kLEDPort};
    std::array<frc::AddressableLED::LEDData, LEDConstants::kPIXEL_COUNT> m_PixelBuffer;

    units::meter_t kLEDSpacing{1.0 / 120.0};
    frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
    frc::LEDPattern m_scrollingRainbox = m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLEDSpacing);

    int m_CurPixelHue = 0;
};