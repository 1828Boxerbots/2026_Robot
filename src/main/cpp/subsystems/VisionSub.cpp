// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSub.h"
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <cameraserver/CameraServer.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cscore_cv.h>

VisionSub::VisionSub()
{
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);

    m_markerSize = 0.2667;
    m_squareNumX = 5;
    m_squareNumY = 7;
    m_squareLength = 0.04;
    m_markerLength = 0.02;
    m_lightThreshold = 175;
    m_darkThreshold = 85;

    frc::SmartDashboard::PutBoolean("Run Camera Calibration", false);

    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    m_charucoBoard = cv::aruco::CharucoBoard(cv::Size(m_squareNumX, m_squareNumY), m_squareLength, m_markerLength, dictionary);

    // cv::FileStorage inputMatrix("VisionConsfigs.txt", cv::FileStorage::READ);
    // inputMatrix["camera_matrix"] >> vCameraMatrix;
    // inputMatrix.release();
    // cv::FileStorage inputDistortion("VisionConsfigs.txt", cv::FileStorage::READ);
    // inputDistortion["distortion_coefficients"] >> vDistCoeffs;
    // inputDistortion.release();


}
VisionSub::~VisionSub()
{}

// This method will be called once per scheduler run
void VisionSub::Periodic() 
{

}



void VisionSub::RunAprilTagDetection()
{
    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-m_markerLength/2.f, m_markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(m_markerLength/2.f, m_markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(m_markerLength/2.f, -m_markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-m_markerLength/2.f, -m_markerLength/2.f, 0);

    cv::Mat frame;

    double distance = 0;

    while (true) {
        cs::CvSink feed = frc::CameraServer::GetVideo();

        feed.GrabFrameNoTimeout(frame);

        // cv::Mat grayFeed;
        // cv::cvtColor(frame, grayFeed, cv::COLOR_BGR2GRAY);

        // cv::Mat dstFeed;
        // // Define the scaling factor (e.g., 0.5 for half size)
        // double scale_factor = 1; 
        // cv::resize(grayFeed, dstFeed, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);

        //  // Apply adaptive thresholding
        // cv::Mat adaptiveThreshFeed, trueLight, trueDark, unsureWhite, unsureGray;
        // int lightThreshold = 170;
        // int darkThreshold = 85;
        // cv::threshold(dstFeed, trueLight,  lightThreshold, 255, cv::THRESH_BINARY);
        // cv::threshold(dstFeed, trueDark,  darkThreshold, 255, cv::THRESH_BINARY);
        // cv::bitwise_xor(trueDark, trueLight, unsureWhite);
        // cv::threshold(unsureWhite, unsureGray, 127, 127, cv::THRESH_BINARY);
        // cv::bitwise_or(trueLight, unsureGray, adaptiveThreshFeed);

        // cv::Mat edgeFeed;
        // cv::Canny(grayFeed, edgeFeed, darkThreshold, lightThreshold);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
        // cv::Mat outputImage = frame.clone();
        // cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        size_t nMarkers = markerCorners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        cv::Mat PosFeed;
        frame.copyTo(PosFeed);
        
        if(!markerIds.empty()) {
            // Calculate pose for each marker
            // std::cout << objPoints << std::endl << markerCorners.at(0) << std::endl << camMatrix << std::endl << distCoeffs << std::endl;
            for (size_t i = 0; i < nMarkers; i++) {
                solvePnP(objPoints, markerCorners.at(i), m_cameraMatrix, m_distCoeffs, rvecs.at(i), tvecs.at(i));
            }

            for(unsigned int i = 0; i < markerIds.size(); i++)
            {
                cv::drawFrameAxes(PosFeed, m_cameraMatrix, m_distCoeffs, rvecs[i], tvecs[i], m_markerSize * 1.5f, 2);

                unsigned int tagId = markerIds[i];

                AprilTagData data
                {
                    std::sqrt((tvecs[i](0) * tvecs[i](0)) + (tvecs[i](1) * tvecs[i](1)) + (tvecs[i](2) * tvecs[i](2)))
                };
                
                m_tagData.insert({tagId, data});
            }
        }
    }
}