#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_header.h"
#include <eStop.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x;
float y;
float phi;
float delta = 0.9;
float pi = 3.14159;


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
}

float distance(float x0,float y0, float x1, float y1){
	float euclidDistance;	
	euclidDistance = sqrt(pow((x1-x0),2)+pow((y1-y0),2));
	return euclidDistance;
}

void pathPlan(vector< vector< float > >& locations){
	int destination=0;
	float minDist=0;
	for (int i = 1; i < locations.size(); i++){	
		if (minDist > distance(locations[i][0],locations[i][1],x,y) && locations[i][3] == 0){
			minDist = (locations[i][0],locations[i][1],x,y);
			destination = i;
		}
		ROS_INFO("distance -> %f; destination-> %d, %d; ",distance(locations[i][0],locations[i][1],x,y),destination,i);
	} 

}	

bool moveToGoal(float xGoal, float yGoal, float phiGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}

int main(int argc, char** argv){
	cv::initModule_nonfree();
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
    teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<vector<float> > orient;
	std::vector<cv::Mat> imgs_track;
	vector<vector<float> > target;	
	init(coord, orient, imgs_track);

	ROS_INFO("x->%f, y->%f",x,y);
	//save desiredd robot positions (modified from box locations)	
	for (int i=0; i<coord.size(); i++){
		coord[i].push_back(orient[i][0]);
		coord[i].push_back(0);
		target.push_back(coord[i]);		
		target[i][0]=target[i][0]+delta*cos(target[i][2]);
		target[i][1]=target[i][1]+delta*sin(target[i][2]);
		if (target[i][2] >= 0){
			target[i][2] = target[i][2] - pi;
		}
		else{
			target[i][2] = target[i][2] + pi;
		}
		ROS_INFO("Box %d -> %f, %f, %f, %f", i, target[i][0],target[i][1],target[i][2], target[i][3]);
		
	}
	pathPlan(target);
    findPic(n, imgs_track);	
    while(ros::ok()){
        //.....**E-STOP DO NOT TOUCH**.......
        eStop.block();
        //...................................

        //fill with your code

		
		
    	}

	return 0;
}
/*
//-------------------------move robot function---------------
bool moveToGoal(float xGoal, float yGoal, float phiGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*//*
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}*/
