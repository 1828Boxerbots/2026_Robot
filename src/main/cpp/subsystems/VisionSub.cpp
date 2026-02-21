// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSub.h"
#include <thread>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <cameraserver/CameraServer.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cscore_cv.h>

std::map<unsigned int, AprilTagData> VisionSub::m_tagData;
double VisionSub::m_translationValue;

VisionSub::VisionSub()
{
    // frc::SmartDashboard::PutBoolean("Confirm Calibration", false);
    frc::SmartDashboard::PutBoolean("Run Cailbration", false);

    std::thread visionThread(VisionThread);
    visionThread.detach();
}
VisionSub::~VisionSub()
{}

// This method will be called once per scheduler run
void VisionSub::Periodic() 
{
    
}

void VisionSub::VisionThread()
{
    bool runCalibration = false;

    if (!runCalibration)
    {
    RunAprilTagDetection();
    }
    else
    {
    RunCharucoBoardCailbration();
    }
}

double VisionSub::GetTagTranslation()
{
    return m_translationValue;
}

void VisionSub::RunAprilTagDetection()
{
    
    cv::Mat camMatrix, distCoeffs;
    cv::FileStorage inputMatrix("/home/lvuser/VisionMatrixConfig.txt", cv::FileStorage::READ);
    inputMatrix["camera_matrix"] >> camMatrix;
    inputMatrix.release();
    cv::FileStorage inputDistortion("/home/lvuser/VisionDistConfig.txt", cv::FileStorage::READ);
    inputDistortion["distortion_coefficients"] >> distCoeffs;
    inputDistortion.release();

        // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    float markerLength = 0.2667;
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    cv::Mat frame;

    double distance = 0;

    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture(0);
    camera.SetResolution(640, 480);
    camera.SetFPS(60);

    cs::CvSource outputStream = frc::CameraServer::PutVideo("Detector", 640, 480);
    outputStream.SetFPS(60);

    cs::CvSink feed = frc::CameraServer::GetVideo("USB Camera 0");

    while (true) {

        if (feed.GrabFrameNoTimeout(frame) == 0)
        {
            std::cout << "Error: Didn't grab the frame" << std::endl;
        }
        else
        {
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
            cv::aruco::ArucoDetector detector(dictionary, detectorParams);
            detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

            size_t nMarkers = markerCorners.size();
            std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
            
            cv::Mat PosFeed = frame.clone();
            
            if(!markerIds.empty()) {
                // Calculate pose for each marker
                // std::cout << objPoints << std::endl << markerCorners.at(0) << std::endl << camMatrix << std::endl << distCoeffs << std::endl;
                for (size_t i = 0; i < nMarkers; i++) {
                    solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                }

                for(unsigned int i = 0; i < markerIds.size(); i++)
                {

                    unsigned int tagId = markerIds[i];

                    AprilTagData data
                    {
                        std::sqrt((tvecs[i](0) * tvecs[i](0)) + (tvecs[i](1) * tvecs[i](1)) + (tvecs[i](2) * tvecs[i](2)))
                    };

                    m_tagData.insert({tagId, data});
                    
                    cv::drawFrameAxes(PosFeed, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
                    cv::aruco::drawDetectedMarkers(PosFeed, markerCorners, markerIds);

                    if((markerIds[i] == 26) || (markerIds[i] == 10))
                    {
                        m_translationValue = tvecs[i](0);
                    }
                }
            }
            
            outputStream.PutFrame(PosFeed);
        }
    }
}

void VisionSub::RunCharucoBoardCailbration()
{   
    cv::Mat board;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    int squareNumX = 5;
    int squareNumY = 7;
    float squareLength = 0.04;
    float markerLength = 0.02;
    cv::aruco::CharucoBoard charucoBoard = cv::aruco::CharucoBoard(cv::Size(squareNumX, squareNumY), squareLength, markerLength, dictionary);
    
    // Collect data from each frame
    std::vector<cv::Mat> allCharucoCorners, allCharucoIds;
    
    std::vector<std::vector<cv::Point2f>> allImagePoints;
    std::vector<std::vector<cv::Point3f>> allObjectPoints;

    std::vector<cv::Mat> allImages;
    cv::Size imageSize;
    cv::Mat image;

    cv::aruco::CharucoDetector charucoDetector(charucoBoard);

    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture(0);
    camera.SetResolution(640, 480);
    camera.SetFPS(60);

    cs::CvSource outputStream = frc::CameraServer::PutVideo("Detector", 640, 480);
    outputStream.SetFPS(60);

    cs::CvSink feed = frc::CameraServer::GetVideo("USB Camera 0");

    bool putingCailbration = true;
    
    do
    {   
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Mat currentCharucoCorners, currentCharucoIds;
        std::vector<cv::Point3f> currentObjectPoints;
        std::vector<cv::Point2f> currentImagePoints;

        if (feed.GrabFrameNoTimeout(image) == 0)
        {
            std::cout << "Error: Didn't grab the frame" << std::endl;
        }
        else
        {
            // std::cout << "It works frfr" << std::endl;
            charucoDetector.detectBoard(image, currentCharucoCorners, currentCharucoIds);
        
            cv::Mat outputImage = image.clone();
        
            if (currentCharucoCorners.total() > 3)
            {
                charucoBoard.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);
                // std::cout << std::endl << "I CAN SEE" << std::endl << std::endl;
                cv::aruco::drawDetectedCornersCharuco(outputImage, currentCharucoCorners, currentCharucoIds);
                if(currentImagePoints.empty() || currentObjectPoints.empty()) 
                {
                    continue;
                }
                allImagePoints.push_back(currentImagePoints);
                allObjectPoints.push_back(currentObjectPoints);
                allImages.push_back(image);
                imageSize = image.size();

                if (putingCailbration)
                {
                    putingCailbration = false;

                    std::cout << "i put cailbration of charuco board" << std::endl;

                    cv::Mat cameraMatrix, distCoeffs;
                    // Calibrate camera using ChArUco
                    double repError = cv::calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray());

                    cv::FileStorage outputDistor("/home/lvuser/VisionDistConfig.txt", cv::FileStorage::WRITE);
                    outputDistor << "distortion_coefficients" << distCoeffs;
                    outputDistor.release();
                    
                    cv::FileStorage outputMatrix("/home/lvuser/VisionMatrixConfig.txt", cv::FileStorage::WRITE);
                    outputMatrix << "camera_matrix" << cameraMatrix;
                    outputMatrix.release();
                }
            }
        
            outputStream.PutFrame(outputImage);
            
        } // for the check if it work

    } while (true); 
}