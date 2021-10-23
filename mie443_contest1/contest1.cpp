#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <eStop.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>


using namespace std;
ros::Publisher vel_pub;

double angular, linear, tolerance = 0.4;
int go, turnCheckl= 0, turnCheckr = 0;
int reversing = 0;
int n;
int wait;
//odom variables
double posX, posY, yaw, yaw_update, yaw_init;
double pi = 3.1416;

//bumper variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
double posXBump = 0, posYBump = 0; 

//laser variables
double laserRange = 10;
double laserRangeL = 10;
double laserRangeR = 10;
float odomx=0, odomy=0, TotalDist=0;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
int turn_direction = 0;
int aboutTurn = 0;



void bumperCallback(const kobuki_msgs::BumperEvent msg){
	//fill with your code
	if(msg.bumper == 0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCenter = !bumperCenter;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;

}

void depthCallback(const sensor_msgs::ImageConstPtr& dmsg){
	//fill with your code
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);

	laserRange = 11;
	laserRangeL = 11;
	laserRangeR = 11;
	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			//Split laser range readings into three sectors
			if(laserRange>msg->ranges[i]){
				laserRange = msg->ranges[i]; //center sector

			}
		}
		for(int j=1; j < laserSize/2 - laserOffset; j++){
			if(laserRangeR>msg->ranges[j]){	
				laserRangeR = msg->ranges[j]; //right sector
			}
		}
		for(int z = laserSize/2 + laserOffset; z < laserSize; z++){
			if(laserRangeL>msg->ranges[z]){
                                laserRangeL = msg->ranges[z]; //left sector
                        }
		}
	}
	
	else{
		for(int i = 1; i < laserSize; i++){
			if(laserRange>msg->ranges[i]){
				laserRange = msg->ranges[i]; //center sector

			}
		}
                for(int j=1; j < laserSize/2 - laserOffset; j++){
                        if(laserRangeR>msg->ranges[j]){
                                laserRangeR = msg->ranges[j]; //right sector
			}
                        
                }
                for(int z = laserSize/2 + laserOffset; z < laserSize; z++){
                        if(laserRangeL>msg->ranges[z]){
                                laserRangeL = msg->ranges[z]; //left sector
			}
		}
	}

	if(laserRange == 11)
		laserRange =0;

        if(laserRangeL == 11)
                laserRangeL =0;

        if(laserRangeR == 11)
                laserRangeR =0;
}


void check30()
/*Checks whether turtlebot has rotated 30 degree increments from last straight motion pose and only sets go condition to true
once 30 degree incremental rotation has finished. If after 3 increments there is still an object too close, we rotate 180 degrees
in the opposite direction*/
{
	if(!go)
	{
		//compare global yaw with reference yaw then increment; included logical operators to deal
		//with both positive and negative angles
		if(abs(yaw_init - yaw) <= pi)
		{
			yaw_update = abs(yaw_init - yaw);
			
		}
		else
		{
			yaw_update = 2*pi-abs(yaw_init - yaw);

		}	
	}
	
	if(aboutTurn){

		if (yaw_update < 5.5*pi/12){
			go = 0;
		}
		else if(yaw_update >= 5.5*pi/12 ){
			aboutTurn = 0;
			n = 1;
			yaw_init = yaw;	
			go = 1;	
		}
		else{
			go = 0;
		}
		
	}
	else if(!aboutTurn && laserRange < 0.7 && yaw_update < pi/6){
		go = 0; //not yet 30 degrees

	}
	else if(!aboutTurn && laserRange < 0.7){
		if(yaw_update < pi/3){ //rotate another 30 degrees (60 degrees total)
			go = 0;
		}
		else if(laserRange < 0.7){
			if(yaw_update < 5*pi/12){ //rotate approx another 30 degrees (approx 90 degrees total)
				go = 0;
			}
			else if(laserRange < 0.7){
				go = 0;
				n = -1; //switch turning direction
				aboutTurn = 1; //turn 180 degrees in the other direction
			}
			else{
				//enough clearance so reset variables to resume straight line motion
				go = 1; 
				yaw_update = 500;
				n = 1;
				yaw_init = yaw;	
			}
		}
		else{
			//enough clearance so reset variables to resume straight line motion
			go = 1;
			yaw_update = 500;
			n = 1;
		}
	}

	else{
		//enough clearance so reset variables to resume straight line motion
		go = 1;
		yaw_update = 500;
		n =1;
	}
	

}

void turnl(double angle)
/*Checks whether turtlebot has turned (both linear and angular motion active) a desired angle counter clockwise
from last straight motion pose and only sets turnCheckl condition to false once the turn is complete*/
{
	if(turnCheckl)
	{
		//compare global yaw with reference yaw then increment; included logical operators to deal
		//with both positive and negative angles
		if(abs(yaw_init - yaw) <= pi)
		{
			yaw_update = abs(yaw_init - yaw);
			
		}
		else
		{
			yaw_update = 2*pi-abs(yaw_init - yaw);

		}	
	}
	
	if(yaw_update < angle*pi/180){
		turnCheckl = 1; //turn not finished
	}
	else{
		//turn is complete so set turnCheckl to 0 and reset yaw_update
		turnCheckl = 0;
		yaw_update = 500;
	}
}


void turnr(double angle)
/*Checks whether turtlebot has turned (both linear and angular motion active) a desired angle clockwise
from last straight motion pose and only sets turnCheckr condition to false once the turn is complete*/
{
	if(turnCheckr)
	{
		//compare global yaw with reference yaw then increment; included logical operators to deal
		//with both positive and negative angles
		if(abs(yaw_init - yaw) <= pi)
		{
			yaw_update = abs(yaw_init - yaw);
			
		}
		else
		{
			yaw_update = 2*pi-abs(yaw_init - yaw);

		}	
	}
	
	if(yaw_update < angle*pi/180){
		turnCheckr = 1; //turn not finished
	}
	else{
		//turn is complete so set turnCheckr to 0 and reset yaw_update
		turnCheckr = 0;
		yaw_update = 500;
	}
}
int main(int argc, char **argv)
{

 	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
    	image_transport::ImageTransport it(nh);
    	teleController eStop;

	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
   	image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/image_raw", 1, depthCallback);

	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);
	ros::Subscriber laser = nh.subscribe("scan", 1, laserCallback);


	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//Initializing variable values
	angular = 0.0;
	linear = 0.0;
	yaw_update = 500;
	yaw_init = yaw;
	go = 1;

	//TotalDist = 0;
	n = 1;

	geometry_msgs::Twist vel;

	while(ros::ok()){
		ros::spinOnce();
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................
	

	// use total distance travelled to switch turning biases.
	if(wait==1000 )	
	{TotalDist = sqrt(pow((posX-odomx),2) + pow((posY-odomy),2)) + TotalDist;
		 odomx = posX;
		  odomy = posY;

	        if(TotalDist<15){
			turn_direction = 1;
		}
		else if(TotalDist<30){
			turn_direction = 0;
		}
		else{
			TotalDist=0;
		}
		wait = 0;
	 }
	 else
	{wait++;

	}	 

		//bumpers
		if(bumperCenter || bumperLeft || bumperRight){
			posXBump = posX;
			posYBump = posY;
			if(!bumperCenter && !bumperRight){reversing = 1;} //hit left
			else if (!bumperLeft && !bumperCenter){reversing = 3;} //hit right
			else {reversing = 2;} //hit center or multiple
		}

		if(reversing != 0){
			linear = -0.15;
			if(reversing == 1){angular = -pi/6;}
			else if(reversing == 2){ angular=-pi/6;}
			else if(reversing == 3){angular = pi/6;}
			if(pow((posX-posXBump),2) + pow((posY-posYBump),2)>0.04){
				reversing = 0; linear=0; angular = 0;
			}
		}

		else if(go && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7 && laserRangeL > 0.7 && laserRangeR > 0.7)
		{
			
			if((laserRangeL > laserRangeR + tolerance && laserRangeL ) || turnCheckl)
				// Possible gap on the left so turn counterclockwise
				{
					turnCheckl = 1;
					turnl(45);
					angular = pi/9;
					linear = 0.15;
				}
			else if((laserRangeR > laserRangeL + tolerance && laserRangeR ) || turnCheckr)
				// Possible gap on the right so turn clockwise
				{
					turnCheckr = 1;
					turnr(45);
					angular = -pi/9;
					linear = 0.15;
				}
			else{
				angular = 0.0;
				linear = 0.2;
				yaw_init = yaw;//save yaw during straight line motion as reference		
			}
		}
		else if(go && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7 && laserRangeL > 0.5 && laserRangeR > 0.7 )
		{
			angular = -pi/10;
			linear = 0.15;
			yaw_init = yaw;//save yaw during straight line motion as reference
		}
		else if(go && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7 && laserRangeL > 0.7 && laserRangeR > 0.5)
		{	angular = +pi/10;
			linear = 0.15;

			yaw_init = yaw;//save yaw during straight line motion as reference
		}
		
		
		else if(!go || !bumperRight && !bumperCenter && !bumperLeft) 
		//If distance to object less than 0.7; rotate
		{	
			if(turn_direction=0){
				go = 0; //not OK to go straight
				angular = n*pi/6;
				//TotalDist = 200000;
			}
			else if(turn_direction=1){
				go = 0; //not OK to go straight
				angular = -n*pi/6;
			}
			else{
				go = 0; //not OK to go straight
				angular = n*pi/6;
			}
			linear = 0.0;
			check30();// check if turtlebot has rotated 30 degrees
		}


		else 
		{        	
			if(turn_direction=0){
			go = 0; //not OK to go straight
			angular = n*pi/6;
			}
			else if(turn_direction=1){
				go = 0; //not OK to go straight
				angular = -n*pi/6;
			}
			else{
				go = 0; //not OK to go straight
				angular = n*pi/6;
			}
			linear = 0.0;

		}
  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
