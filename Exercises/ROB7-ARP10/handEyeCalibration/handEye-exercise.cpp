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
#include <iterator>
#include <sstream>
#include <fstream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;

// The following split function thannks to 
// https://stackoverflow.com/questions/236129/how-do-i-iterate-over-the-words-of-a-string

template <typename Out>
void split(const std::string& s, char delim, Out result) {
	std::istringstream iss(s);
	std::string item;
	while (std::getline(iss, item, delim)) {
		*result++ = item;
	}
}

std::vector<std::string> split(const std::string& s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, std::back_inserter(elems));
	return elems;
}

struct ExtrinsicCoords
{
	float x;
	float y;
	float z;
	float rx;
	float ry;
	float rz;
};


int main()
{
	// Read camera calibration parameters
	FileStorage fs("cameraParams.yml", FileStorage::READ);

	Mat cameraMatrix, distCoeffs;
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	std::vector<Mat> rvecs, tvecs;
	fs["rvecs"] >> rvecs;
	fs["tvecs"] >> tvecs;

	// Step 1: Convert rotation vectors to matrix format
	// Tip: Use OpenCV Rodrigues function
	

	// Step 2: Read the rotation and translation matrices from gripper frame
	// to robot base frame
	// You get this code for free :)
	std::ifstream coordinateFile;
	coordinateFile.open("coordinates.txt", std::ios::in);
	std::string line;
	bool firstLine = true;

	std::map<int, ExtrinsicCoords> gripperCoords;

	if (coordinateFile.is_open()) 
	{
		while (std::getline(coordinateFile, line)) 
		{
			if (firstLine)
			{
				firstLine = false;
				continue;
			}

			std::vector<std::string> coords = split(line, ',');

			if (coords.size() >= 7)
			{
				ExtrinsicCoords gripperCoord;
				gripperCoord.x = std::stof(coords[1]); // Convert string to float
				gripperCoord.y = std::stof(coords[2]);
				gripperCoord.z = std::stof(coords[3]);
				gripperCoord.rx = std::stof(coords[4]);
				gripperCoord.ry = std::stof(coords[5]);
				gripperCoord.rz = std::stof(coords[6]);

				gripperCoords[std::stoi(coords[0])] = gripperCoord;
			}
		}
	}

	// Step 3: Convert the code from our own struct to a OpenCV-friendly format


	// Step 4: Calculate the actual hand-eye calibration using 
	// calibrateHandEye


	// Step 5: Convert the rotation matrix to a vector such that we 
	// may interpret the results
	// Tip: Use OpenCV Rodrigues function



	// Step 6: Print the final results
	//int x;
	//std::cout << "Rot: " << std::endl << rotVecCam2Gripper << std::endl;
	//std::cout << "Trans: " << std::endl << translationCamera2Gripper << std::endl;
	//std::cin >> x ;
}