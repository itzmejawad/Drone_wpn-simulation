//MIT License
//
//Copyright (c) 2019 Aalborg University
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

// Code for the course 'Advanced Robotic Perception' in Robotics 7th semester
#include <string>
#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std::filesystem;


int main()
{
	// The following part is shamelessly adapted from https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html

	std::string path = "../../UR kalibrering/chessboard/";
	Size imageSize;
	std::map<int, Mat> calibImages;

	for (const auto& entry : directory_iterator(path))
	{
		if (entry.path().extension().string().find(".png") != std::string::npos) {
			auto img = imread(entry.path().string());

			auto basename = entry.path().stem(); // Returns the file name without the extension

			if (img.cols > 0) {
				calibImages[std::stoi(basename)] = img;
				imageSize = img.size();
			}
		}
	}

	std::vector<std::vector<Point2f> > imagePoints;
	Size boardSize(8, 6);
	float squareSize = 12.5;

	if (!calibImages.empty())
	{
		for (auto const& [imageId, calibImage] : calibImages)
		{
			std::vector<Point2f> pointBuffer;
			bool found = findChessboardCorners(calibImage, boardSize, pointBuffer,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

			if (found)
			{
				Mat greyImage;
				cvtColor(calibImage, greyImage, COLOR_BGR2GRAY);
				cornerSubPix(greyImage, pointBuffer, Size(11, 11), Size(-1, -1), // zeroZone addition
					TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

				imagePoints.push_back(pointBuffer);
			}

			Mat drawImage = calibImage.clone();
			drawChessboardCorners(drawImage, boardSize, Mat(pointBuffer), found);
			//imshow("Detected points", drawImage);
			//waitKey(200);

		}

		// Run the actual calibration
		std::vector<Mat> rvecs, tvecs;
		std::vector<std::vector<Point3f> > objectPoints(1);


		// Populate the objectPoints according to standard chessboard setup
		for (int i = 0; i < boardSize.height; ++i) {
			for (int j = 0; j < boardSize.width; ++j) {
				objectPoints[0].push_back(Point3f(j * squareSize, i * squareSize, 0));
			}
		}

		objectPoints.resize(imagePoints.size(), objectPoints[0]);

		Mat cameraMatrix;
		Mat distCoeffs;

		double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,
			rvecs, tvecs);

		std::cout << "Reprojection error: " << rms << std::endl;

		// Save the parameters
		FileStorage fs("cameraParams.yml", FileStorage::WRITE);
		fs << "cameraMatrix" << cameraMatrix;
		fs << "distCoeffs" << distCoeffs;
		fs << "rvecs" << rvecs;
		fs << "tvecs" << tvecs;
		fs.release();

		// Okay, so we have officially calibrated the camera and acquired the intrinsic and extrinsic camera parameters. 
		// Let's try to create 3D effects on those calibration images

		std::string windowName = "3D chessboard image";

		int visualizedImageId = 1;
		Mat visualizedImage = calibImages[visualizedImageId].clone();
		imshow(windowName, visualizedImage);
		

		if (visualizedImage.empty()) {
			return 1;
		}

		Mat currentRVec = rvecs[visualizedImageId - 1];
		Mat currentTVec = tvecs[visualizedImageId - 1];;


		
		// Create the 3D points for the illustration that we want to make
		// First, we will try to show the x-y-z axis on top of the chessboard
		// Step a-1: Create the 3D coordinates for the x-y-z axes
		// Tip: The dataformat should be in std::vector<Point3f>
		

		// Step a-2: Project 3D points to image plane
		

		// Step a-3: Draw the axes (lines) on top of the 2D image
		

		// Then we will draw a 3D cube
		// Step b-1: Define the 3D coordinates of the cube in the chessboard reference frame
		

		// Step b-2: Project the cube 3D points to image space
		

		// Step b-2: Draw the actual cube

		waitKey(0);

	}
}