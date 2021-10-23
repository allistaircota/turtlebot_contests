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
  image_transport::Subscriber sub = it.subscribe("camera/image/", 1, imageCallback); //--for the webcam
  //image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback); //--for the kinect


  int foundPic, count1 =0 ,count2=0;
  
  cv::Mat video;
  cv::Mat grey;
  Mat descriptors_object, descriptors_scene;
  Mat img_matches, H;
while(ros::ok()){ 
 count1++;
 
//cout << count1%20000 << endl;
  ros::spinOnce();
  if(!img.empty()){
	  img.copyTo(video);
	  cvtColor(video, grey, CV_BGR2GRAY);
	  //img.release();
	 
  	  int minHessian = 400;

 	  SurfFeatureDetector detector( minHessian );

  	  std::vector<KeyPoint> keypoints_object, keypoints_scene;
	  detector.detect( imgs_track[0], keypoints_object );
	  detector.detect( grey, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
	  SurfDescriptorExtractor extractor;


          extractor.compute( imgs_track[0], keypoints_object, descriptors_object );
	  extractor.compute(grey, keypoints_scene, descriptors_scene );  
	  FlannBasedMatcher matcher;
	  std::vector< DMatch > matches;
	  cout << "img search: " << imgs_track[0].empty() << endl;
	  cout << "1: " << descriptors_object.empty() << endl;
	  cout << "2: " << descriptors_scene.empty() << endl; 
	  if(descriptors_scene.empty()==0){	 
	  matcher.match( descriptors_object, descriptors_scene, matches );
	
	  double max_dist = 0; double min_dist = 100;
	
	  //-- Quick calculation of max and min distances between keypoints
	  for( int i = 0; i < descriptors_object.rows; i++ )
	  { double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	  }

	  printf("-- Max dist : %f \n", max_dist );
	  printf("-- Min dist : %f \n", min_dist );

	  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	  std::vector< DMatch > good_matches;
	   for( int i = 0; i < descriptors_object.rows; i++ )
	  { if( matches[i].distance < 3*min_dist )
	     { good_matches.push_back( matches[i]); }
	  }


	  drawMatches( imgs_track[0], keypoints_object, grey , keypoints_scene,
	               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	  //-- Localize the object
	  std::vector<Point2f> obj;
	  std::vector<Point2f> scene;

	  for( int i = 0; i < good_matches.size(); i++ )
	  {
	    //-- Get the keypoints from the good matches
	    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
	    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	  }

 	  H = findHomography( obj, scene, CV_RANSAC );

	  //-- Get the corners from the image_1 ( the object to be "detected" )
	  std::vector<Point2f> obj_corners(4);
	  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( imgs_track[0].cols, 0 );
	  obj_corners[2] = cvPoint( imgs_track[0].cols, imgs_track[0].rows ); obj_corners[3] = cvPoint( 0, imgs_track[0].rows );
	  std::vector<Point2f> scene_corners(4);

	  perspectiveTransform( obj_corners, scene_corners, H);
	cout << "2" << endl;

	  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
	  line( img_matches, scene_corners[0] + Point2f( imgs_track[0].cols, 0), scene_corners[1] + Point2f( imgs_track[0].cols, 0), Scalar(0, 255, 0), 4 );
	  line( img_matches, scene_corners[1] + Point2f( imgs_track[0].cols, 0), scene_corners[2] + Point2f( imgs_track[0].cols, 0), Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[2] + Point2f( imgs_track[0].cols, 0), scene_corners[3] + Point2f( imgs_track[0].cols, 0), Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[3] + Point2f( imgs_track[0].cols, 0), scene_corners[0] + Point2f( imgs_track[0].cols, 0), Scalar( 0, 255, 0), 4 );
	  cout<<"corner_size"<< scene_corners[0] <<endl;
	  //-- Show detected matches
	  // imshow( "view", img_matches );
	 // imshow( "Good Matches & Object detection", grey );
	cout << "img_matches_empty" << img_matches.size() << endl;
     cv::imshow("view", img_matches);
           }

	  cv::waitKey(5);
	  img.release();
  }
}
  return foundPic;
}
