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

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;


void main()
{
	// Read camera calibration parameters. 
	// The intrinsic camera parameters are enough for this task
	FileStorage fs("cameraParams.yml", FileStorage::READ);

	Mat cameraMatrix, distCoeffs;
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	Mat rubiks;
	rubiks = imread("../../UR kalibrering/terning2.png");

	// Step 1: Define world coordinates
	// Now prompt the user to select the points that defines one of the sides of the cube
	// Define the coordinates in a 3D reference that you define. 
	// Tip1: Use the reference frame (3D axes) provided on the slides
	// Tip2: Use std::vector<Point3f> worldCubePos = { Point3f(x,y,z), ...


	// Step 2: Prompt the user for the corresponding image coordinates
	// This part contains some boiler-plate code, so we leave it here. 
	std::vector<Point2f> imageCubePos;

	for (const auto& worldCoord : worldCubePos) {
		Mat overlay = rubiks.clone();
		std::string pos = "(" + std::to_string(int(worldCoord.x)) + "," + std::to_string(int(worldCoord.y)) + "," + std::to_string(int(worldCoord.z)) + ")";
		putText(overlay, "Mark position: " + pos + " . Select enter to finish", Point(10, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));

		auto roi = selectROI(overlay, true, true);
		
		float centerX = roi.x + roi.width / 2;
		float centerY = roi.y + roi.height / 2;
		imageCubePos.push_back(Point2f(centerX, centerY));
	}

	// Comment or uncomment this part to store and retrieve the image points
	//FileStorage fsCoord("rubiksCoords.yml", FileStorage::WRITE);
	//fsCoord << "imageCubePos" << imageCubePos;
	//fsCoord.release();

	//FileStorage fscoord("rubiksCoords.yml", FileStorage::READ);
	//fscoord["imageCubePos"] >> imageCubePos;
	

	// Step 3: Solve the object pose from the provided 3D to 2D point correspondances
	// You should use the cameraMatrix and distCoeffs that we have previously loaded.

	// Step 4: Check if the object pose is correct by drawing the 3D world frame axes on
	// top of the image. 
	// Tip: Use the OpenCV drawFrameAxes 

	// Now paint the upper side of the cube in green
	// Step 5: Define the world coordinates of the four sides of the polygon that we want to draw
	// If you want to paint the upper side of the cube, the points should reside in the xy-plane

	// Step 6: Convert from world to image coordinates

	// Step 7: And draw the sides as a filled polygon
	
}