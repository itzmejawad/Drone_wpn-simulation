#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Image.h"
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

#define PI 3.14159265
float gx=0;
float gy=0;
float px=0;
float py=0;
float pz=0;
float vx=0;
float vy=0;
float vz=0;
float wx=0;
float wy=0;
float wz=0;
vector<unsigned char> data;
int height = 0;
int width=0;
// calculation of sigma norm of vectors


void handle_poses2(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gx= msg->latitude;
  gy= msg->longitude;
}

void handle_poses1(const nav_msgs::Odometry::ConstPtr& msg)
{
	px= msg->pose.pose.position.x;
	py= msg->pose.pose.position.y;
 	pz= msg->pose.pose.position.z;
}
void handle_vel1(const nav_msgs::Odometry::ConstPtr& msg)
{
  vx= msg->twist.twist.linear.x;
  vy= msg->twist.twist.linear.y;
  vz= msg->twist.twist.linear.z;
  wx= msg->twist.twist.angular.x;
  wy= msg->twist.twist.angular.y;
  wz= msg->twist.twist.angular.z;
}

double deg2rad(double input)
  {
    return input * PI/180;
  }

void capt_image(const sensor_msgs::Image::ConstPtr& imgp)
{
  data= imgp->data;
  height= imgp->height;
  width= imgp->width;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone");
  string m="uav";
  stringstream number;
  ros::NodeHandle n;

  ros::Subscriber gps_pos = n.subscribe("/fix", 10, handle_poses2);
  ros::Subscriber position1 = n.subscribe("/ground_truth/state", 10, handle_poses1);
  ros::Subscriber velocitlat = n.subscribe("/ground_truth/state", 10, handle_vel1);

  ros::Subscriber get_img = n.subscribe("/downward_cam/camera/image", 10, capt_image);
  ros::Publisher desired_velocitlat= n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::ServiceClient motors_enable1 = n.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
  if (!motors_enable1.waitForExistence(ros::Duration(5.0)))
  {
    ROS_WARN("Motor1 enable service not found");
    return false;
  }

  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;
  motors_enable1.call(srv);
  //motors_enable2.call(srv);
  //motors_enable3.call(srv);

  ros::Rate loop_rate(100);
  geometry_msgs::Twist msg1;
  ros::Time begin = ros::Time::now();

  vector<double> ilon;
  vector<double> ilat;
  vector<double> iz1;
  //vector<float> iz1;
  std::vector<std::string> row;

  float desx=0;
  float desy=0;
  float desz=0;
  float disst=0;
  //float ilon[1] = {};
  //float ilon[2]={8.686, 8.687007};//waypoints
  //float ilat[2]={49.860, 49.860246};

  fstream fin;
    fin.open("/home/sarvesh/drone_ws/src/drone_wpn/src/Waypoints.csv");
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
  
  
        }
        cout << "right:" <<row[0] << ",ledt:" << row[1] << endl;
        double lat = std::stod(row[0]);
        double lon = std::stod(row[1]);
        double ht = std::stod(row[2]);
        ilon.push_back(lon);  
        ilat.push_back(lat);
        iz1.push_back(ht);
        cout << setprecision(16);
        //iz1.push_back(10);
        }

        

  
  float gy_position = 0;
  float gy_target = 0;
  float gx_position = 0;
  float gx_target = 0;

  float R = 6371000;

  float delta_x = 0;
  float delta_y = 0;
  int i=0;


  while (ros::ok())
  {  

    

    gy_position = deg2rad(gy);
    gy_target=deg2rad(ilon[i]);

    gx_position = deg2rad(gx);
    gx_target=deg2rad(ilat[i]);  



    delta_x = gx_target - gx_position;
    delta_y = gy_target - gy_position;

    desx = delta_x * cos((gy_position + gy_target)/2) * R;
    desy = delta_y * R;
    desz= (iz1[i] -pz);


    
    disst= sqrt(pow(desx,2)+pow(desy,2)+pow(desz,2));
    //desy= (ilon[0] -gy)/disst; 
    //desx= (ilat[0] -gx)/disst; 


    //desx= desx*1000;
    //desy= desy*1000;

    cout << "target lon: "<<ilon[i] << endl;
    cout << "position lon: "<<gy << endl;
    cout << "velocity y: "<<desy << endl;
    

    cout << "target lat: "<<ilat[i] << endl;
    cout << "positon lat: "<<gx << endl;
    cout << "velocity x: "<<desx << endl;

    cout<< "target ht: "<<iz1[i]<<endl;
    cout<< "positon ht: "<<pz<<endl;
    cout << "velocity ht: "<< desz<<endl;

    cout << disst << endl;

    cv::Mat matrix = cv::Mat(height, width, CV_8UC1, data.data());

    float d_max=5;
    float d_min=0.5;
    float d;
    if(disst<d_max)
    { 
      d = 0.1 + (disst - d_min) * (0.9/(d_max - d_min));
      desx= d*desx;
      desy= d*desy;
      desz= d*desz;
      if (disst < d_min)
      {
        desx=0;
        desy=0;
        desz=0;
        if (vx < 0.01 && vy < 0.01 && vz < 0.01)
        {
          ros::Duration(0.5).sleep();
          cv::imwrite("img"+to_string(i)+".jpg", matrix);
          i = i+1;
        }
      }
      
    }
    //if(i>=2)
    //{
      //desx=0;
      //desy=0;
      //desz=0;
      //i=i-1;
    //}

    msg1.linear.x = desx;
    msg1.linear.y = -desy;
    msg1.linear.z = desz;
    ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);
  
    desired_velocitlat.publish(msg1);
    ros::spinOnce();
    loop_rate.sleep();
}
return 0;
}


//roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch