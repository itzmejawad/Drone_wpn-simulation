#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "rosgraph_msgs/Clock.h"
#include "hector_uav_msgs/EnableMotors.h"
#include <iostream>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <string>
#include <sstream>
#include <math.h>
#include <cmath>
#include<algorithm> 

#include <fstream>
//#include <vector>
//#include <iterator>
using namespace std;


int main(int argc, char **argv)
{
  vector<double> ilon;
  vector<double> ilat;
  vector<float> iz1;
  std::vector<std::string> row;
  fstream fin;
  int n = ilon.size();
  fin.open("/home/sarvesh/drone_ws/src/drone_wpn/src/waypoint.csv");
  cout << "reached" << endl;
  string temp, line, word;
  while (!fin.eof())
  {
    row.clear();
    //cout << "reached" << endl;
    getline(fin, line);
    stringstream s(line);
    //cout << line << endl;
    while (getline(s, word, ','))
    {
      //cout << word << endl;
      row.push_back(word);
      
      //row.push_back(word);
    }

   //if (row.size() == 2)
   //{
     //cout << "lat:" << row[0] << ",lon:" << row[1] << endl;
   //}

    double lon = std::stod(row[0]);
    double lat = std::stod(row[1]);
    ilon.push_back(lon);
    ilat.push_back(lat);

    cout << ilon.size() << endl;
    cout << setprecision(16);
    //cout << "right:" << ilon[4] << ",left:" << ilat[4] << endl;
    cout << "right:" << ilon[0] << ",left:" << ilat[0] << endl;
    //cout << "right:" << ilon[0]*100 << endl;
    iz1.push_back(5);
    cout << ilon[0] << endl;
    cout << ilat[0] << endl;
    cout << iz1[0] << endl;

  }

  }
