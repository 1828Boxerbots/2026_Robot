// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <opencv2/opencv.hpp>

class VisionSub : public frc2::SubsystemBase {
 public:
  VisionSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  bool mRunCalibration;
  int mSquareNumX = 5;
  int mSquareNumY = 7;
  float mSquareLength = 0.04;
  float mMarkerLength = 0.02;
  cv::Mat mBoard;
  std::vector<cv::Mat> mAllCharucoCorners, mAllCharucoIds;
  std::vector<std::vector<cv::Point2f>> mAllImagePoints;
  std::vector<std::vector<cv::Point3f>> mAllObjectPoints;
  std::vector<cv::Mat> mAllImages;
  cv::Size mImageSize;
  cv::Mat mImage;
};
