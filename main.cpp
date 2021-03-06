#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// #define USE_VIDEO
#define USE_IMAGES

// Defining the dimensions of checkerboard
// int CHECKERBOARD[2]{8,6}; 
// int CHECKERBOARD[2]{6,8}; 
// int CHECKERBOARD[2]{7,7}; 
// int CHECKERBOARD[2]{10,8}; 
int CHECKERBOARD[2]{6,8}; 

bool is_fisheye_model = true;


int main(int argc, char* argv[])
{

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;
    objpoints.clear();

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i = 0; i < CHECKERBOARD[1]; i++) {
        for(int j = 0; j < CHECKERBOARD[0]; j++) {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;
    imgpoints.clear();

    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    corner_pts.clear();


    
    cv::Mat frame, ori, gray;
    int count            = 0;
    int frameCount       = 0;
    size_t frameInterval = 1;

    std::string videoPath = "/Users/leonlin/Downloads/Camera Calibration data_20211210/20200102182218_000003AA.MP4"; 
    
    // Path of the folder containing checkerboard images with postfix of image type
    std::string imgPath = "/Users/leonlin/Downloads/abeo_AG190C_calib_img/*.png"; 


    #ifdef USE_VIDEO
    // Path of the video containing checkerboard images
    cv::VideoCapture cap(videoPath);

    if (!cap.isOpened()) {
        std::cout << "VideoCapture is not opened" << std::endl;
    }


    while (cap.isOpened())
    {   
        if (!cap.read(frame)) break;
        std::cout << "frameCount = " << frameCount++ << std::endl;
        ori = frame.clone();
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        bool success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
    
        /* 
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display 
         * them on the images of checker board
         */
        if(success) {
            ++count;
            if (frameCount%frameInterval != 0) continue;
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
      
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
      
            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
      
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);

            cv::imwrite("../success/"+std::to_string(frameCount)+".jpg", frame);
            cv::imwrite("../success_ori/"+std::to_string(frameCount)+".jpg", ori);
        }
        cv::imshow("video",frame);
        cv::waitKey(1);
    }
    cap.release();
    #endif // USE_VIDEO


    #ifdef USE_IMAGES
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;


    cv::glob(imgPath, images);

    for(size_t i = 0; i < images.size(); i++) {
        frame = cv::imread(images[i]);
        std::cout << images[i] << std::endl;
        ori = frame.clone();
        cv::cvtColor(frame,gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        bool success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
    
        /* 
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display 
         * them on the images of checker board
         */
        if(success) {
            ++count;
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
      
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1), criteria);
      
            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
      
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        cv::imshow("video",frame);
        cv::waitKey(1);
    }
    #endif // USE_IMAGES
    std::cout << "\n\nsuccess number of frame for calibration = " << count << std::endl;

    /*
     * Performing camera calibration by 
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the 
     * detected corners (imgpoints)
     */
    cv::Mat cameraMatrix,distCoeffs,R,T;
    if (!is_fisheye_model) {
        cv::calibrateCamera(objpoints, imgpoints, ori.size(), cameraMatrix, distCoeffs, R, T);
    } else {
        std::cout << "Use cv::fisheye::calibrate()" << std::endl;
        int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC|cv::fisheye::CALIB_FIX_SKEW;
        cv::TermCriteria criteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, 30, 1e-6);
        cv::fisheye::calibrate(objpoints, imgpoints, ori.size(), cameraMatrix, distCoeffs, R, T, flags, criteria);
    }
    

    std::cout << "cameraMatrix = " << cameraMatrix << std::endl << std::endl
              << "distCoeffs = " << distCoeffs << std::endl << std::endl;
            //   << "R = " << R << std::endl 
            //   << "T = " << T << std::endl; 

    /*
     * Estimate New Camera Matrix For Undistort Rectify,
     * which can be used via pin-hole model
     */
    cv::Mat OptimalK, map1, map2;
    if (!is_fisheye_model) {   
        OptimalK = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, ori.size(), true);
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs,  cv::Mat(), OptimalK, ori.size(), CV_16SC2, map1, map2);
    } else {
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, ori.size(), cv::Mat(), OptimalK, true);
        cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), OptimalK, ori.size(), CV_16SC2, map1, map2);
    }
    std::cout << "OptimalK = " << OptimalK << std::endl << std::endl;


    // cv::Mat source = cv::imread("../1634961801400.jpg");
    // if (source.empty()) {
    //     std::cout << "source is not found!" << std::endl;
    // } else {
    //     cv::Mat undistort, result;
    //     cv::remap(source, undistort, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    //     cv::vconcat(source, undistort, result);
    //     cv::imshow("calibrated_result", result);
    //     cv::waitKey(0);
    // }
    return 0;
}
