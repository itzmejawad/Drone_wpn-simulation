#!/usr/bin/env python

from __future__ import print_function

import sys, select, termios
from std_msgs.msg import Header, Empty
import copy
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, Point, PoseStamped
import hector_uav_msgs
import math



desired_pos_x = desired_pos_y = desired_pos_z = 0;
velocity_x = velocity_y = velocity_z = 0;
wx = wy = wz = 0;

def position1(data):
	px = data.pose.pose.position.x
	py = data.pose.pose.position.y 
	pz = data.pose.pose.position.z 

def handle_velocity(data):
	velocity_x = data.twist.twist.linear.x
	velocity_y = data.twist.twist.linear.y
	velocity_z = data.twist.twist.linear.z
	wx = data.twist.twist.angular.x
	wy = data.twist.twist.angular.y
	wz = data.twist.twist.angular.z

if __name__=="__main__":

	position1 = rospy.Subscriber("/ground_truth/state", Odometry, position1)
	velocity1 = rospy.Subscriber("/ground_truth/state", Odometry, handle_velocity)
	desired_velocity = rospy.Publisher("/uav/cmd_vel_unstamped",Twist,queue_size=100)
	#msg1 = rospy.Publisher()
	#motors_enable1 = rospy.Service("/uav1/mavros/cmd/arming", CommandBool)
	rospy.init_node('drone')

	#if (!motors_enable1.waitForExistence(rospy.sleep(5.0))):
	#	rospy.warn("Motors are shit")
	#	return false

#hector_uav_msgs.EnableMotors(true)
#motors_enable1.call

rospy.Rate (100)
#geometry_msgs.twist(msg1)
msg1 = Twist()
begin = rospy.get_rostime()
desx = desy = desz =0
disst = 0
ilat = [41.0824324]
ilon = [29.0501797]
iheight = [15]

i = 0

while not rospy.is_shutdown():
	disst = math.sqrt(pow((ilat[i] -desired_pos_x), 2) + pow((ilon[i] -desired_pos_y), 2)+pow((iheight[i] -desired_pos_z), 2))
	desx = (ilat[i] - desired_pos_x)/disst
	desy = (ilon[i] - desired_pos_y)/disst
	desz = (iheight[i] - desired_pos_z)/disst

	if (disst<1):
		desx = 0.1*desx
		desy = 0.1*desy
		desz = 0.1*desz
		i = i + 1

	if (i>=4):
		desx=0
    	desy=0
    	desz=0	
    	i = i-1

    
	msg1.angular.x = desx
	msg1.linear.y = desy
	msg1.linear.z = desz
	#rospy.loginfo(msg1.linear.x,msg1.linear.y,msg1.linear.z)

	desired_velocity.publish(msg1)
	rospy.spin()
	#rospy.sleep()


#roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch
#rosservice call \enable_motors true






####
geometry_msgs::PoseStamped pose_;

joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1,
                                                                 boost::bind(&Teleop::joyPoseCallback, this, _1));

      pose_.pose.position.x = 0;
      pose_.pose.position.y = 0;
      pose_.pose.position.z = 0;



####