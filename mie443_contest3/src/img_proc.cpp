#include "nav_header.h"

using namespace cv;
cv::Mat img;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
	img = cv_bridge::toCvShare(msg, "bgr8")->image;

  }catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    img.release();
  }
}
 
int findPic(ros::NodeHandle nh, vector<cv::Mat> imgs_track){

	cv::namedWindow("view");
	cv::startWindowThread();
  	image_transport::ImageTransport it(nh);
  	//image_transport::Subscriber sub = it.subscribe("camera/image/", 1, imageCallback); //--for the webcam
  	image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback); //--for the kinect


  	int foundPic=0, count1 =0 ,count2=0;
  
  	cv::Mat video;
  	cv::Mat grey;
  	Mat descriptors_object, descriptors_scene;
  	Mat img_matches, H;
  	while(ros::ok()){
			  img.release();
 		while(!img.data || img.empty())
		{	img.release();
			ros::spinOnce();
		}

  		if(!img.empty() && img.data){	
			foundPic =0;				
			cvtColor(img, grey, CV_BGR2GRAY);
  		  	int minHessian = 400;
	 	  	SurfFeatureDetector detector( minHessian );
		  	std::vector<KeyPoint> keypoints_object, keypoints_scene;
			detector.detect( grey, keypoints_scene );
		  	SurfDescriptorExtractor extractor;
			extractor.compute(grey, keypoints_scene, descriptors_scene );  
			for (int i=0;i<imgs_track.size();i++){
		  		detector.detect( imgs_track[i], keypoints_object );
		  		//-- Step 2: Calculate descriptors (feature vectors)
          			extractor.compute( imgs_track[i], keypoints_object, descriptors_object );
		  		FlannBasedMatcher matcher;
		  		std::vector< DMatch > matches;
		  		matcher.match( descriptors_object, descriptors_scene, matches );		
		  		double max_dist = 0; double min_dist = 100;
		
		  		//-- Quick calculation of max and min distances between keypoints
		  		for( int j = 0; j < descriptors_object.rows; j++ )
		  		{
					double dist = matches[j].distance;
		    			if( dist < min_dist ) min_dist = dist;
		    			if( dist > max_dist ) max_dist = dist;
		  		}
	
		  		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		  		std::vector< DMatch > good_matches;
		   		for( int k = 0; k < descriptors_object.rows; k++ )
		  		{
					if( matches[k].distance < 3*min_dist )
		     			{
						good_matches.push_back( matches[k]);
					}
		  		}
	
		  		drawMatches( imgs_track[i], keypoints_object, grey , 	keypoints_scene,good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	
		  		//-- Localize the object
		  		std::vector<Point2f> obj;
		  		std::vector<Point2f> scene;
	
		  		for( int l = 0; l < good_matches.size(); l++ )
		  		{
		    			//-- Get the keypoints from the good matches
		    			obj.push_back( keypoints_object[ good_matches[l].queryIdx ].pt );
		    			scene.push_back( keypoints_scene[ good_matches[l].trainIdx ].pt );
		  		}

 		  		H = findHomography( obj, scene, CV_RANSAC );

		  		//-- Get the corners from the image_1 ( the object to be "detected" )
		  		std::vector<Point2f> obj_corners(4);
		  		obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( imgs_track[0].cols, 0 );
		  		obj_corners[2] = cvPoint( imgs_track[0].cols, imgs_track[0].rows ); obj_corners[3] = cvPoint( 0, imgs_track[0].rows );
		  		std::vector<Point2f> scene_corners(4);	
	  			perspectiveTransform( obj_corners, scene_corners, H);
	
	  			//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	  			line( img_matches, scene_corners[0] + Point2f( imgs_track[i].cols, 0), scene_corners[1] + Point2f( imgs_track[i].cols, 0), Scalar(0, 255, 0), 4 );
	  			line( img_matches, scene_corners[1] + Point2f( imgs_track[i].cols, 0), scene_corners[2] + Point2f( imgs_track[i].cols, 0), Scalar( 0, 255, 0), 4 );
	  			line( img_matches, scene_corners[2] + Point2f( imgs_track[i].cols, 0), scene_corners[3] + Point2f( imgs_track[i].cols, 0), Scalar( 0, 255, 0), 4 );
	  			line( img_matches, scene_corners[3] + Point2f( imgs_track[i].cols, 0), scene_corners[0] + Point2f( imgs_track[i].cols, 0), Scalar( 0, 255, 0), 4 );
				float ratio1 = (pow(scene_corners[0].x - scene_corners[1].x,2) + pow( scene_corners[0].y - scene_corners[1].y,2)) / (pow(scene_corners[2].x - scene_corners[3].x,2) + pow( scene_corners[2].y - scene_corners[3].y,2)) ;
          			float ratio2 = (pow(scene_corners[1].x - scene_corners[2].x,2) + pow( scene_corners[1].y - scene_corners[2].y,2)) / (pow(scene_corners[3].x - scene_corners[0].x,2) + pow( scene_corners[3].y - scene_corners[0].y,2)) ;
          			float ratio_cross = (pow(scene_corners[1].x - scene_corners[3].x,2) + pow( scene_corners[1].y - scene_corners[3].y,2)) / (pow(scene_corners[2].x - scene_corners[0].x,2) + pow( scene_corners[3].y - scene_corners[0].y,2)) ;
				float cross_length=(pow(scene_corners[1].x - scene_corners[3].x,2) + pow( scene_corners[1].y - scene_corners[3].y,2));
				cout<< "cross_length"<< cross_length <<endl;

	  			if(ratio1 <4 && ratio1>0.25 && ratio2 < 4 && ratio2 > 0.25  && ratio_cross < 4 && ratio_cross > 0.25 && cross_length > 10000 )
	  			{
	  				foundPic = i+1;
	  			}
				//-- Show detected matches
				
	  			//cv::imshow("view", img_matches);
	        		//cv::waitKey(1000);

	  
	  	 	 }
			  img.refcount = 0;
			  img.release();
			  grey.refcount = 0;
			  grey.release();
			  img_matches.refcount = 0;
			  img_matches.release();
			  return foundPic;		
  	 	 }
  	}
  return 0;
}
