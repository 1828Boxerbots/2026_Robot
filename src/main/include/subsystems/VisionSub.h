// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/opencv.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <thread>
#include <string>
#include <queue>


class VisionSub : public frc2::SubsystemBase {
 public:
  VisionSub();
  ~VisionSub();

  void RunCharucoBoardCailbration();
  void RunAprilTagDetection();
  void VisionThread();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Network tables
  nt::NetworkTableInstance inst;
  nt::DoublePublisher testPub;
  std::vector<nt::DoubleArrayPublisher> publishers;
  double idData[9];
};