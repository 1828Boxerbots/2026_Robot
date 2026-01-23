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

VisionSub::VisionSub() = default;

// This method will be called once per scheduler run
void VisionSub::Periodic() 
{

    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);
    cs::CvSink frame = frc::CameraServer::GetVideo();
    cs::CvSource outputFeed = frc::CameraServer::PutVideo("video", 640, 480);

    // cs::CvSink sink = cs::GrabFrameNoTimeout(&mImage);

    frame.GrabFrameNoTimeout(mImage);

    mRunCalibration = frc::SmartDashboard::GetBoolean("Run Camera Calibration", false);
    if (mRunCalibration)
    {
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        cv::aruco::CharucoBoard charucoBoard = cv::aruco::CharucoBoard(cv::Size(mSquareNumX, mSquareNumY), mSquareLength, mMarkerLength, dictionary);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Mat currentCharucoCorners, currentCharucoIds;
        std::vector<cv::Point3f> currentObjectPoints;
        std::vector<cv::Point2f> currentImagePoints;
        cv::aruco::CharucoDetector charucoDetector(charucoBoard);
        charucoDetector.detectBoard(mImage, currentCharucoCorners, currentCharucoIds);

        if(currentCharucoCorners.total() > 5) 
        {
            // Match image points
            charucoBoard.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);
 
            if(!currentImagePoints.empty() && !currentObjectPoints.empty()) 
            {
                mImageSize = mImage.size();

                cv::Mat cameraMatrix, distCoeffs;
                // Calibrate camera using ChArUco
                double repError = cv::calibrateCamera(mAllObjectPoints, mAllImagePoints, mImageSize, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray());
                
                cv::FileStorage outputDistor("VisionConsfigs.txt", cv::FileStorage::WRITE);
                outputDistor << "distortion_coefficients" << distCoeffs;
                outputDistor.release();

                cv::FileStorage outputMatrix("VisionConsfigs.txt", cv::FileStorage::WRITE);
                outputMatrix << "camera_matrix" << cameraMatrix;
                outputMatrix.release();
            }
        }
    }
}


/*
    cv::Mat board;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    int squareNumX = 5;
    int squareNumY = 7;
    float squareLength = 0.04;
    float markerLength = 0.02;
    cv::aruco::CharucoBoard charucoBoard = cv::aruco::CharucoBoard(cv::Size(squareNumX, squareNumY), squareLength, markerLength, dictionary);
    charucoBoard.generateImage(cv::Size(500, 700), board);

    cv::VideoCapture cap(0); // Open the default camera
    
    // Collect data from each frame
    std::vector<cv::Mat> allCharucoCorners, allCharucoIds;
    
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allObjectPoints;

    std::vector<cv::Mat> allImages;
    cv::Size imageSize;
    cv::Mat image;

    cv::aruco::CharucoDetector charucoDetector(charucoBoard);

    do
    {
        cap >> image;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Mat currentCharucoCorners, currentCharucoIds;
        std::vector<cv::Point3f> currentObjectPoints;
        std::vector<cv::Point2f> currentImagePoints;

        charucoDetector.detectBoard(image, currentCharucoCorners, currentCharucoIds);
        if(cv::waitKey(1) == 'c' && currentCharucoCorners.total() > 3) {
            // Match image points
            charucoBoard.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);
            std::cout << "Detecting Image..." << std::endl;
            cv::Mat outputImage = image.clone();
            cv::aruco::drawDetectedCornersCharuco(outputImage, currentCharucoCorners, currentCharucoIds);
            cv::imshow("output Image", outputImage);
 
            if(currentImagePoints.empty() || currentObjectPoints.empty()) {
                std::cout << "Point matching failed, try again." << std::endl;
                continue;
            }
            allImagePoints.push_back(currentImagePoints);
            allObjectPoints.push_back(currentObjectPoints);
            allImages.push_back(image);
            imageSize = image.size();
        }
        else if (cv::waitKey(1) == 'c') { std::cout << "Cant detect board" << std::endl;}

        cv::imshow("Image", image);
        cv::imshow("Aruco", board);

    } while (cv::waitKey(1) != 'q');
    
    cv::Mat cameraMatrix, distCoeffs;
    // Calibrate camera using ChArUco
    double repError = cv::calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,
                                      cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray());
    if(cameraMatrix.empty() || distCoeffs.empty()) 
    {
        std::cout << "Failed calibration" << std::endl;
    }
    if(!cameraMatrix.empty() || !distCoeffs.empty()) 
    {
        std::cout << "Cailbration success" << std::endl;
    }
    cv::FileStorage outputDistor("C:/Users/njpat/OneDrive/Documents/Distortion.txt", cv::FileStorage::WRITE);
    // outputDistor << "camera_matrix" << cameraMatrix;
    outputDistor << "distortion_coefficients" << distCoeffs;
    outputDistor.release();

    cv::FileStorage outputMatrix("C:/Users/njpat/OneDrive/Documents/Matrix.txt", cv::FileStorage::WRITE);
    outputMatrix << "camera_matrix" << cameraMatrix;
    // outputMatrix << "distortion_coefficients" << distCoeffs;
    outputMatrix.release();

*/