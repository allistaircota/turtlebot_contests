#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <kobuki_msgs/BumperEvent.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <cmath>
#include <eStop.h>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>

#include "nav_header.h"
#include <sound_play/sound_play.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace cv;
geometry_msgs::Twist follow_cmd;
int world_state;
double sad_start;
//laser variables
double laserRange = 10;
double laserRangeL = 10;
double laserRangeR = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
bool bumperOn = 0;

//odom variables
double posX, posY, yaw, yaw_update, yaw_init;
const double pi = 3.1416;

//tuning parameters
const double linearOffset = 0.07;
const double angularOffset = 0.07;
const double laserDelta = 0.5;

//emotion parameters
double last_time = 0;
int sadCount = 0;
int sadSound = 0;
int sadTime = 0;
double rageCount = 0;
double rageWaitLimit = 9;
int sadWaitLimit = 4;
double sad_duration = 0;

double rage_movement2_start;
double shake_start;
double rage_movement_start;
double shake_start2; 
double surprise_start; 
double spin_start;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);

	laserRange = 11;
	laserRangeL = 11;
	laserRangeR = 11;
	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(laserRange>msg->ranges[i]){
				laserRange = msg->ranges[i];
			}
		}
		for(int j=1; j < laserSize/2 - laserOffset; j++){
			if(laserRangeR>msg->ranges[j]){	
				laserRangeR = msg->ranges[j];
			}
		}
		for(int z = laserSize/2 + laserOffset; z < laserSize; z++){
			if(laserRangeL>msg->ranges[z]){
                                laserRangeL = msg->ranges[z];
                        }
		}
	}
	
	else{
		for(int i = 1; i < laserSize; i++){
			if(laserRange>msg->ranges[i]){
				laserRange = msg->ranges[i];

			}
		}
                for(int j=1; j < laserSize/2 - laserOffset; j++){
                        if(laserRangeR>msg->ranges[j]){
                                laserRangeR = msg->ranges[j];
			}
                }
                for(int z = laserSize/2 + laserOffset; z < laserSize; z++){
                        if(laserRangeL>msg->ranges[z]){
                                laserRangeL = msg->ranges[z];
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

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
    if(msg.bumper == 0 || msg.bumper == 1 || msg.bumper == 2)
		{bumperOn = !bumperOn;}
}

void depthCallback(const sensor_msgs::ImageConstPtr& dmsg){	
	//Fill with code
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg){
	//Fill with code
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    sound_play::SoundClient sc;
    string path_to_sounds = "/home/turtlebot/catkin_ws/src/mie443_contest3/src/sounds/";
    teleController eStop;

    image_transport::ImageTransport it(nh);

	//subscribers
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
    image_transport::Subscriber rgb_sub = it.subscribe("camera/rgb/image_raw", 1, rgbCallback);
	image_transport::Subscriber depth_sub = it.subscribe("camera/depth_registered/image_raw", 1, depthCallback); //--for the kinect
	//image_transport::Subscriber sub = it.subscribe("camera/image/", 1, imageCallback); //--for the webcam
	ros::Subscriber laser = nh.subscribe("scan", 1, laserCallback);

	//publishers
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

    double angular = 0.0;
    double linear = 0.0;
    
    // Image & OpenCV initialization 
    std::vector<cv::Mat> imgs_track;
    imgs_track.push_back(cv::imread( "/home/turtlebot/catkin_ws/src/mie443_contest3/src/img/XJ-9_grayscale_square.jpg"));
    
    // Velocity command initialization 
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    Mat image_normal = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/img/normal.png", CV_LOAD_IMAGE_COLOR);
    Mat image_angry = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/img/angry.png", CV_LOAD_IMAGE_COLOR);
    Mat image_sad = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/img/sad.png", CV_LOAD_IMAGE_COLOR);
    Mat image_love = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/img/love.png", CV_LOAD_IMAGE_COLOR);
    Mat image_surprise = imread("/home/turtlebot/catkin_ws/src/mie443_contest3/src/img/surprise_alt.png", CV_LOAD_IMAGE_COLOR);
   // Read the file

    if(! image_normal.data || ! image_angry.data || ! image_love.data || ! image_sad.data)   // Check for invalid input
    { 
        cout <<  "Could not open or find the image" << std::endl ;
	return 0;
    }
    else{
    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image_normal );                   // Show our image inside it.
    waitKey(5);
    
    }

    while(ros::ok()){
        ros::spinOnce();
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................
		if(bumperOn){
			world_state = 2;bumperOn = 0;}
		else if(abs(follow_cmd.linear.x) < linearOffset && abs(follow_cmd.angular.z) < angularOffset){ // check if follow_cmd velocity close to zero
			world_state = 1;
		}
		else{
			world_state = 0;
		}
		
    	if(world_state == 2){
    		// Surprised!
    		world_state = 0;
    		imshow( "Display window", image_surprise); // Show image          
       		sc.playWave(path_to_sounds + "Surprise.wav"); // Sound play
       		waitKey(5);
       		 // Drive control
    		vel.angular.z = 0;
       		vel.linear.x = -2;
    		vel_pub.publish(vel);

    		surprise_start = ros::Time::now().toSec();
    		while(ros::Time::now().toSec() - surprise_start<0.2) { }// delay 
       		vel.linear.x = 0;
    		vel_pub.publish(vel);
    		while(ros::Time::now().toSec() - surprise_start<2) { }// delay
       		imshow( "Display window", image_normal );
                       // Show our image inside it.
       		waitKey(5);
    			sadSound = 1;
				sadTime = 1;
    	}
           else if(world_state == 0){
                //fill with your code
                vel_pub.publish(follow_cmd); // Neutral state, follow person
    			sadSound = 1;
				sadTime = 1;
    
            }
    	else if(world_state == 1){
            // Can't find target to follow or target is not moving
    
        	if(laserRange > 1 - laserDelta && laserRange < 1 + laserDelta/3){
                	if(last_time == 0 || ros::Time::now().toSec() - last_time > rageWaitLimit+0.2 ){
        		    last_time = ros::Time::now().toSec();
        		}
        		
        		else if(floor(rageCount) == 2){
        			int foundIT = findPic(nh,imgs_track); // Perform image detection
				//int foundIT = 1;
        			rageCount = ros::Time::now().toSec() - last_time; 
        			if(foundIT != 0){
        			    // Infatuation!
        			    imshow( "Display window", image_love ); // Show image
           			    sc.playWave(path_to_sounds + "Infatuation1.wav"); // Play sound
           			    waitKey(5);
					   spin_start = ros::Time::now().toSec();
    					while(ros::Time::now().toSec() - spin_start<2.45) {vel.angular.z = pi;
       				    vel.linear.x = 0;
		       		    vel_pub.publish(vel);}// delay 
    				    vel.angular.z = 0;
       				    vel.linear.x = 0;
		       		    vel_pub.publish(vel);
        			    rageCount = 0;	
        			    last_time = 0;
				//    cout<<"1111111111111111111111         "<<ros::Time::now().toSec()<<endl;
        			}
				else{sc.playWave(path_to_sounds + "horn.mp3"); sleep(0.05);}
        			sadSound = 1;
					sadTime = 1;
        		}
        		
        		else if(rageCount < rageWaitLimit){
        			ROS_INFO("Rage wait time %f", rageCount);
        			rageCount = ros::Time::now().toSec() - last_time; 
        			imshow( "Display window", image_normal ); // Show image
          			waitKey(5);
        			sadSound = 1;
					sadTime = 1;	
        		}
        		
        		else{	
        			ROS_INFO("Rage");
        			sc.playWave(path_to_sounds + "Rage.wav");
        			imshow( "Display window", image_angry ); // Show image
          			waitKey(5);
        			// Screen stuff
        			// Movement stuff

        			rage_movement_start= ros::Time::now().toSec();
				int sign = 1;
        			while(ros::Time::now().toSec() - rage_movement_start <1.5){
					   cout <<"time"<< ros::Time::now().toSec()<<"     "<<rage_movement_start<<endl; 
        				vel.angular.z = pi/2 * sign;
        				vel.linear.x = 0;
        				vel_pub.publish(vel);
        				sign = -1*sign;
					   shake_start = ros::Time::now().toSec();
    					while(ros::Time::now().toSec() - shake_start<0.1) { }// delay 
        			}
        			rage_movement2_start= ros::Time::now().toSec();
				   sign = 1;
        			while(ros::Time::now().toSec() - rage_movement2_start <3){
        				vel.angular.z = 0;
        				vel.linear.x = 0.3*sign;
        				vel_pub.publish(vel);
        				sign = -1*sign;

						shake_start2 = ros::Time::now().toSec();
    					while(ros::Time::now().toSec() - shake_start2<0.5) {
						ros::spinOnce();
						if(bumperOn){
							vel.angular.z = 0;
							vel.linear.x = 0.8*sign;
							sign = -1*sign;
							bumperOn = 0;
							sleep(0.5);
						}
					 }// delay 
        			}
				vel.angular.z = 0;
        			vel.linear.x = 0;
        			vel_pub.publish(vel);
        			
        			last_time = 0;
        			rageCount = 0;	
        			imshow( "Display window", image_normal ); // Show image
          			waitKey(5);
        			sadSound = 1;
					sadTime = 1;	
        		}
        		
        		
        	}
    		else{
				// Save sad_start as current time
				if (sadTime == 1){
				 	sad_start = ros::Time::now().toSec();
					sadTime = 0;
				}
    			if(sad_duration <4){
    				sad_duration = ros::Time::now().toSec() - sad_start;
    			}
    			else if(sad_duration >4){
    				cout <<"sad"<<sad_duration<<endl;
    				sad_duration = 0;
    				sad_start = ros::Time::now().toSec();
    				if (sadSound == 1){
    					sc.playWave(path_to_sounds + "Sad2.mp3");
					 	imshow( "Display window", image_sad );                   // Show our image inside it.
    				}
    				sadSound = 0;
					sadTime = 0;

      				waitKey(5);

    			}
    			else{
					  sad_duration =0;
    			     sad_start = ros::Time::now().toSec(); 
    			}
    			
    			// Screen stuff
    			sadCount = 0;
			last_time = 0;
        		rageCount = 0;	
	    		vel.angular.z = 0.0;
	    		vel.linear.x = 0.05;
	    		vel_pub.publish(vel);
    			
    		}					
    
        } 
    }
	return 0;
}
