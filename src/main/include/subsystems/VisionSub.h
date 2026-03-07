// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <opencv2/opencv.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <mutex>

struct AprilTagData
{
  double distance;
  double rot;
  double shootVelocity;
};

// template <>
// struct wpi::Struct<AprilTagData> 
// {
//   static constexpr std::
// };

class VisionSub : public frc2::SubsystemBase {
 public:
  VisionSub();
  ~VisionSub();

  static void RunCharucoBoardCailbration();
  static void RunAprilTagDetection();
  static void VisionThread();

  static double GetTagTranslation();
  static double GetTagDistance();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  static double m_translationValue;
  static double m_shootVelocity;

  nt::DoubleArrayPublisher publisher;
  // nt::NetworkTableEntry IdData;

  std::shared_ptr<nt::NetworkTable> visionTable;
  nt::NetworkTableInstance inst;

  static std::map<unsigned int, AprilTagData> m_tagData;
};