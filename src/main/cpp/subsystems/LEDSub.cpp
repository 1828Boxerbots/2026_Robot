#include "subsystems/LEDSub.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

LEDSub::LEDSub()
{

}

void LEDSub::Init()
{
    m_AddressableLED.SetLength(LEDConstants::kPIXEL_COUNT);
    m_AddressableLED.SetData(m_PixelBuffer);
    m_AddressableLED.Start();

    // auto allianceColor = frc::DriverStation::GetAlliance();
    // if (allianceColor.value() == frc::DriverStation::Alliance::kRed)
    // {
    //     SetRangeRGB(0, OperatorConstants::PIXEL_COUNT - 1, 255, 0, 0);
    // }
    // else
    // {
    //     SetRangeRGB(0, OperatorConstants::PIXEL_COUNT - 1, 0, 0, 255);
    // }
}

void LEDSub::Periodic()
{
    // if (frc::DriverStation::IsDisabled())
    // {
    //     auto allianceColor = frc::DriverStation::GetAlliance();
    //     if (allianceColor.value() == frc::DriverStation::Alliance::kRed)
    //     {
    //         SetRangeRGB(0, OperatorConstants::PIXEL_COUNT - 1, 255, 0, 0);
    //     }
    //     else
    //     {
    //         SetRangeRGB(0, OperatorConstants::PIXEL_COUNT - 1, 0, 0, 255);
    //     }

    //     ApplyBuffer();
    // }

    for (int i = 0; i < LEDConstants::kPIXEL_COUNT; i++)
    {
        const auto pixelHue = (m_CurPixelHue + (i * 180 / LEDConstants::kPIXEL_COUNT));
        m_PixelBuffer[i].SetHSV(pixelHue, 255, 128);
    }

    m_CurPixelHue += 3;
    m_CurPixelHue %= 180;

    ApplyBuffer();

    // std::cout << "Buffer Size: " << m_PixelBuffer.size();
    // m_scrollingRainbox.ApplyTo(m_PixelBuffer);
    // m_AddressableLED.SetData(m_PixelBuffer);
}

void LEDSub::SetPixelRGB(unsigned int i, uint8_t r, uint8_t g, uint8_t b)
{
    // Only index into allowed range
    if (i >= LEDConstants::kPIXEL_COUNT)
        return;

    m_PixelBuffer[i].SetRGB(r, g, b);
}

void LEDSub::SetRangeRGB(unsigned int start, unsigned int end, uint8_t r, uint8_t g, uint8_t b)
{
    // Only index into allowed range
    if (start >= LEDConstants::kPIXEL_COUNT)
        return;

    // end must be after start and within valid range
    if (end <= start || end >= LEDConstants::kPIXEL_COUNT)
        return;

    for (int i = start; i <= end; i++)
    {
        m_PixelBuffer[i].SetRGB(r, g, b);
    }
}

void LEDSub::ApplyBuffer()
{
    std::cout << m_PixelBuffer.size() << std::endl;
    m_AddressableLED.SetData(m_PixelBuffer);
}