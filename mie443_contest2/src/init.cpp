#include "nav_header.h"

void init(vector<vector<float> >& coord, vector<vector<float> >& orient, std::vector<cv::Mat>& imgs_track){
	
	//set up coord Vector
	coord.clear();
	float c1[2] = {1.857, -0.153};
	float c2[2] = {1.9744, 0.4719};
	float c3[2] = {0.4078, 0.9061};
	float c4[2] = {0.4458, -0.069};

	coord.push_back(vector<float>(c2, c2 + sizeof c2 / sizeof c2[0]));
	coord.push_back(vector<float>(c1, c1 + sizeof c1 / sizeof c1[0]));
	coord.push_back(vector<float>(c3, c3 + sizeof c3 / sizeof c3[0]));
	coord.push_back(vector<float>(c4, c4 + sizeof c4 / sizeof c4[0]));

	//set up orient vector
	orient.clear();
	float o1[2] = {-0.7889};
	float o2[2] = {0.9873};
	float o3[2] = {0.17725};
	float o4[2] = {0.32333};

	orient.push_back(vector<float>(o2, o2 + sizeof o2 / sizeof o2[0]));
	orient.push_back(vector<float>(o1, o1 + sizeof o1 / sizeof o1[0]));
	orient.push_back(vector<float>(o3, o3 + sizeof o3 / sizeof o3[0]));
	orient.push_back(vector<float>(o4, o4 + sizeof o4 / sizeof o4[0]));
	
	imgs_track.clear();
	//setup images for search
	imgs_track.push_back(cv::imread( "/home/turtlebot/Pictures/Webcam/tag1.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	imgs_track.push_back(cv::imread( "/home/turtlebot/Pictures/Webcam/tag2.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	imgs_track.push_back(cv::imread( "/home/turtlebot/Pictures/Webcam/tag3.jpg", CV_LOAD_IMAGE_GRAYSCALE));
}
