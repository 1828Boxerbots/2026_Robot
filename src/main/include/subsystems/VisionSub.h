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
  ~VisionSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  bool m_runCalibration;
  int m_squareNumX;
  int m_squareNumY;
  float m_squareLength;
  float m_markerLength;
  cv::Mat m_board;
  std::vector<cv::Mat> m_allCharucoCorners, m_allCharucoIds;
  std::vector<std::vector<cv::Point2f>> m_allImagePoints;
  std::vector<std::vector<cv::Point3f>> m_allObjectPoints;
  std::vector<cv::Mat> m_allImages;
  cv::Size m_imageSize;
  cv::Mat m_feed;
  cv::Mat m_distCoeffs, m_cameraMatrix;
  float m_markerSize;
  cv::Mat m_grayFeed;
  cv::Mat m_edgeFeed;
  int m_lightThreshold;
  int m_darkThreshold;
  cv::Mat m_posFeed;
  cv::aruco::CharucoBoard m_charucoBoard;
  struct  AprilTagData
  {
    // value is in meters
    double distance;
  };

  std::map<unsigned int, AprilTagData> m_tagData;
};