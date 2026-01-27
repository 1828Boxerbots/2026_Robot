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

    // cs::CvSource outPut = frc::CameraServer::PutVideo("Camera Feed", 640, 480);
    // cv::Mat mat = new Mat();
    // frame.GrabFrame(mat);
    // outPut.PutFrame(mat);

    // std::cout << "pos1" << std::endl;

    m_runCalibration = frc::SmartDashboard::GetBoolean("Run Camera Calibration", false);

    // std::cout << "pos2" << std::endl;
    if (m_runCalibration)
    {   

        // std::cout << "pos3" << std::endl;

        // cs::CvSink frame = frc::CameraServer::GetVideo();

        // std::cout << "pos4" << std::endl;

        // frame.GrabFrame(m_feed, 0.01);

        // std::cout << "pos5" << std::endl;

        // if (!m_feed.empty())
        //     {
        //     std::vector<int> markerIds;
        //     std::vector<std::vector<cv::Point2f>> markerCorners;
        //     cv::Mat currentCharucoCorners, currentCharucoIds;
        //     std::vector<cv::Point3f> currentObjectPoints;
        //     std::vector<cv::Point2f> currentImagePoints;
        //     cv::aruco::CharucoDetector charucoDetector(m_charucoBoard);
        //     charucoDetector.detectBoard(m_feed, currentCharucoCorners, currentCharucoIds);

        //     if(currentCharucoCorners.total() > 3)
        //     {
        //         std::cout << "detected Charuco Board" << std::endl;
        //     }
        // }   

    //     if(currentCharucoCorners.total() > 5) 
    //     {
    //         // Match image points
    //         charucoBoard.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);
 
    //         if(!currentImagePoints.empty() && !currentObjectPoints.empty()) 
    //         {
    //             m_imageSize = m_feed.size();

    //             cv::Mat cameraMatrix, distCoeffs;
    //             // Calibrate camera using ChArUco
    //             double repError = cv::calibrateCamera(m_allObjectPoints, m_allImagePoints, m_imageSize, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray());
                
    //             cv::FileStorage outputDistor("VisionConsfigs.txt", cv::FileStorage::WRITE);
    //             outputDistor << "distortion_coefficients" << distCoeffs;
    //             outputDistor.release();

    //             cv::FileStorage outputMatrix("VisionConsfigs.txt", cv::FileStorage::WRITE);
    //             outputMatrix << "camera_matrix" << cameraMatrix;
    //             outputMatrix.release();
    //         }
    //     }
    }

    // //April tag detection scope
    // {
    //     cv::Mat objPoints(4, 1, CV_32FC3);
    //     objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-m_markerLength/2.f, m_markerLength/2.f, 0);
    //     objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(m_markerLength/2.f, m_markerLength/2.f, 0);
    //     objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(m_markerLength/2.f, -m_markerLength/2.f, 0);
    //     objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-m_markerLength/2.f, -m_markerLength/2.f, 0);

    //     std::vector<int> markerIds;
    //     std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    //     cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    //     cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    //     cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    //     detector.detectMarkers(m_feed, markerCorners, markerIds, rejectedCandidates);
    //     cv::Mat outputImage = m_feed.clone();
    //     cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    //     // cs::CvSource outputImage = frc::CameraServer::PutVideo("IdImage", 640, 480);

    //     size_t nMarkers = markerCorners.size();
    //     std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
    //     cv::Mat PosFeed;
    //     m_feed.copyTo(PosFeed);

    //     if(!markerIds.empty()) {
    //         // Calculate pose for each marker
    //         // std::cout << objPoints << std::endl << markerCorners.at(0) << std::endl << camMatrix << std::endl << distCoeffs << std::endl;
    //         for (size_t i = 0; i < nMarkers; i++) {
    //             solvePnP(objPoints, markerCorners.at(i), m_cameraMatrix, m_distCoeffs, rvecs.at(i), tvecs.at(i));
    //         }

    //         m_tagData.clear();

    //         for(unsigned int i = 0; i < markerIds.size(); i++)
    //         {
    //             cv::drawFrameAxes(PosFeed, m_cameraMatrix, m_distCoeffs, rvecs[i], tvecs[i], m_markerSize * 1.5f, 2);

    //             unsigned int tagId = markerIds[i];

    //             AprilTagData data
    //             {
    //                 std::sqrt((tvecs[i](0) * tvecs[i](0)) + (tvecs[i](1) * tvecs[i](1)) + (tvecs[i](2) * tvecs[i](2)))
    //             };

    //             m_tagData.insert({tagId, data});
    //         }
    //         cv::aruco::drawDetectedMarkers(PosFeed, markerCorners, markerIds);
    //         // if(cv::waitKey(1) == 'c')
    //         // {
    //         //     std::cout<<tvecs[0]<<std::endl;//<<rvecs[0]<<std::endl;
    //         //     mDistance = std::sqrt((tvecs[0](0) * tvecs[0](0)) + (tvecs[0](1) * tvecs[0](1)) + (tvecs[0](2) * tvecs[0](2)));
    //         //     std::cout << mDistance << std::endl;
    //         // }
    //     }
    // }
}

/*
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

        cv::Mat camMatrix, distCoeffs;
        cv::FileStorage inputMatrix("C:/Users/njpat/OneDrive/Documents/Matrix.txt", cv::FileStorage::READ);
        inputMatrix["camera_matrix"] >> camMatrix;
        // inputMatrix << "camera_matrix" << camMatrix;
        inputMatrix.release();
        cv::FileStorage inputDistortion("C:/Users/njpat/OneDrive/Documents/Distortion.txt", cv::FileStorage::READ);
        inputDistortion["distortion_coefficients"] >> distCoeffs;
        // inputDistortion << "distortion_coefficients"" << distCoeffs;
        inputDistortion.release();
        // std::cout << camMatrix << std::endl << distCoeffs << std::endl;

            // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    float markerLength = 0.2667;
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    cv::Mat frame;

    double distance = 0;

    while (true) {
        cap >> frame; // Capture a new frame

        if (frame.empty()) {
            std::cerr << "Error: Captured empty frame." << std::endl;
            break;
        }

        cv::Mat grayFeed;
        cv::cvtColor(frame, grayFeed, cv::COLOR_BGR2GRAY);

         // Apply adaptive thresholding
        cv::Mat adaptiveThreshFeed, trueLight, trueDark, unsureWhite, unsureGray;
        int lightThreshold = 170;
        int darkThreshold = 85;
        cv::threshold(dstFeed, trueLight,  lightThreshold, 255, cv::THRESH_BINARY);
        cv::threshold(dstFeed, trueDark,  darkThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_xor(trueDark, trueLight, unsureWhite);
        cv::threshold(unsureWhite, unsureGray, 127, 127, cv::THRESH_BINARY);
        cv::bitwise_or(trueLight, unsureGray, adaptiveThreshFeed);

        cv::Mat edgeFeed;
        cv::Canny(grayFeed, edgeFeed, darkThreshold, lightThreshold);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);
        cv::Mat outputImage = frame.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        size_t nMarkers = markerCorners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        cv::Mat PosFeed;
        frame.copyTo(PosFeed);
        
        if(!markerIds.empty()) {
            // Calculate pose for each marker
            // std::cout << objPoints << std::endl << markerCorners.at(0) << std::endl << camMatrix << std::endl << distCoeffs << std::endl;
            for (size_t i = 0; i < nMarkers; i++) {
                solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }

            for(unsigned int i = 0; i < markerIds.size(); i++)
            {
                cv::drawFrameAxes(PosFeed, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);

            }
            cv::aruco::drawDetectedMarkers(PosFeed, markerCorners, markerIds);
            if(cv::waitKey(1) == 'c')
            {
                std::cout<<tvecs[0]<<std::endl;//<<rvecs[0]<<std::endl;
                distance = std::sqrt((tvecs[0](0) * tvecs[0](0)) + (tvecs[0](1) * tvecs[0](1)) + (tvecs[0](2) * tvecs[0](2)));
                std::cout << distance << std::endl;
            }
        }

        cv::imshow("Camera Feed", frame);
        //cv::imshow("Light & Dark Camera Feed", adaptiveThreshFeed);
        //cv::imshow("Altered Camera Feed", edgeFeed);
        cv::imshow("AprilTagDetection Camera Feed", outputImage);
        cv::imshow("3dPos Camera Feed", PosFeed);


        if (cv::waitKey(1) == 'q') { // Wait for 'q' key to exit
            break;
        }
    }
*/