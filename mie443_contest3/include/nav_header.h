#ifndef NAV_HEADER_H
#define NAV_HEADER_H

#include <ros/console.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

#include <vector>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

int findPic(ros::NodeHandle nh, vector<cv::Mat> imgs_track);

void init(vector<vector<float> >& coord, vector<vector<float> >& orient, std::vector<cv::Mat>& imgs_track);

#endif
