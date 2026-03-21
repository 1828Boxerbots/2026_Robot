// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSub.h"
#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cscore_cv.h>


VisionSub::VisionSub()
{
    inst = nt::NetworkTableInstance::GetDefault();
    nt::DoubleTopic testTopic = inst.GetDoubleTopic("/Test/X");
    testPub = testTopic.Publish();
    testPub.SetDefault(0.0);

    // visionTable->PutDoubleArray("ID Data", 0.0);

    idData[0] = 0.0; // x
    idData[1] = 0.0; // y
    idData[2] = 0.0; // z
    idData[3] = 0.0; // yaw ?
    idData[4] = 0.0; // pitch ?
    idData[5] = 0.0; // roll ?
    idData[6] = 0.0; // ditance in meters from tag
    idData[7] = 0.0; // x value for tag to center of frame (Not in distance)
    idData[8] = 0.0;  // velocity ball leaving shooter needs to be

    for(int i = 1; i <= 32; i++)
    {
        std::string topicName = "/Vision/Id" + std::to_string(i);
        nt::DoubleArrayPublisher pub = inst.GetDoubleArrayTopic(topicName).Publish();
        pub.SetDefault(idData);
        
        publishers.push_back(std::move(pub));
    }

    

    std::thread visionThread(
        [&]()
        {
            VisionThread();
        }
    );
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

void VisionSub::RunAprilTagDetection()
{
    
    cv::Mat camMatrix, distCoeffs;
    cv::FileStorage inputMatrix("/home/lvuser/VisionMatrixConfig.yml", cv::FileStorage::READ);
    inputMatrix["camera_matrix"] >> camMatrix;
    inputMatrix.release();
    cv::FileStorage inputDistortion("/home/lvuser/VisionDistConfig.yml", cv::FileStorage::READ);
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
    camera.SetBrightness(25);
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
            cv::Mat PosFeed = frame.clone();
            cv::aruco::drawDetectedMarkers(PosFeed, markerCorners, markerIds);

            size_t nMarkers = markerCorners.size();
            std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

            idData[0] = 0.0; // x
            idData[1] = 0.0; // y
            idData[2] = 0.0; // z
            idData[3] = 0.0; // yaw ?
            idData[4] = 0.0; // pitch ?
            idData[5] = 0.0; // roll ?
            idData[6] = 0.0; // ditance in meters from tag
            idData[7] = 0.0; // x value for tag to center of frame (Not in distance)
            idData[8] = 0.0;  // velocity ball leaving shooter needs to be        

            // Zero out network tags
            for(int i = 0; i < 32; i++)
            {
                publishers[i].Set(idData);
            }
            
            if(!markerIds.empty()) {
                // Calculate pose for each marker
                // std::cout << objPoints << std::endl << markerCorners.at(0) << std::endl << camMatrix << std::endl << distCoeffs << std::endl;
                for (size_t i = 0; i < nMarkers; i++) { 
                    // std::cout << objPoints << std::endl << markerCorners.at(i) << std::endl <<  camMatrix << std::endl << distCoeffs << std::endl;
                    cv::solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                }

                for(unsigned int i = 0; i < markerIds.size(); i++)
                {

                    unsigned int tagId = markerIds[i];

                    cv::aruco::drawDetectedMarkers(PosFeed, markerCorners, markerIds);


                    double distance = std::sqrt((tvecs[i](0) * tvecs[i](0)) + (tvecs[i](1) * tvecs[i](1)) + (tvecs[i](2) * tvecs[i](2)));
                    double translationValue = (tvecs[i](0) / tvecs[i](2));

                    double m_numeratorCalculation = (VisionConstants::kGravity * (std::pow(distance, 2.0)));
                    double m_angleCalculation = 2 * (std::pow (cos (VisionConstants::kLaunchAngle), 2));
                    double m_heightCalculation = (VisionConstants::kShooterHeight + (distance * std::tan(VisionConstants::kLaunchAngle)) - VisionConstants::kHubHeight);
                    double shootVelocity = std::sqrt(m_numeratorCalculation / (m_angleCalculation * m_heightCalculation));

                    idData[0] = tvecs[i](0); // x
                    idData[1] = tvecs[i](1); // y
                    idData[2] = tvecs[i](2); // z
                    idData[3] = rvecs[i](0); // yaw ?
                    idData[4] = rvecs[i](1); // pitch ?
                    idData[5] = rvecs[i](2); // roll ?
                    idData[6] = distance; // ditance in meters from tag
                    idData[7] = translationValue; // x value for tag to center of frame (Not in distance)
                    idData[8] = shootVelocity; // velocity ball leaving shooter needs to be

                    publishers[tagId -1].Set(idData);
                }
            }
            
            outputStream.PutFrame(PosFeed);
        }
    }
}

void VisionSub::RunCharucoBoardCailbration()
{   
    std::cout << "run calibration" << std::endl;
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
            std::cout << "Error: Didn't grab the frame :3" << std::endl;
        }
        else
        {
            // std::cout << "It works frfr" << std::endl;
            charucoDetector.detectBoard(image, currentCharucoCorners, currentCharucoIds);
        
            cv::Mat outputImage = image.clone();
        
            if (currentCharucoCorners.total() > 8)
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

                    try
                    {
                        cv::FileStorage outputDistor("/home/lvuser/VisionDistConfig.yml", cv::FileStorage::WRITE);
                        outputDistor << "distortion_coefficients" << distCoeffs;
                        outputDistor.release();

                        cv::FileStorage outputMatrix("/home/lvuser/VisionMatrixConfig.yml", cv::FileStorage::WRITE);
                        outputMatrix << "camera_matrix" << cameraMatrix;
                        outputMatrix.release();
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                }
            }
        
            outputStream.PutFrame(outputImage);
            
        } // for the check if it work

    } while (true); 
}