#include "cornerdetector.h"

static const std::string OPENCV_WINDOW_1 = "Corner Detection";
int flag = 0;
int maxCorners = 48;
bool read_dispImg = false;
bool cornersDetected = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
std::vector<cv::Mat> colorImageVect;
std::vector<cv::Mat> tempColorImageVect;
std::vector <cv::Mat> disparityImageVect;
cv::Mat_<double> qMatrix;
std::vector<cv::Point2f> corners;
std::vector<geometry_msgs::Point> outputMessages(maxCorners);
std::vector<geometry_msgs::Point> GroundTruth(maxCorners);

int once =1;
CornerDetector::CornerDetector(ros::NodeHandle nh) {
    m_image_sub = nh.subscribe("/multisense_sl/camera/left/image_rect_color", 1, &CornerDetector::imageCB, this);
    m_dispimg_sub = nh.subscribe("/multisense_sl/camera/disparity",10,&CornerDetector::dispCB,this);
    m_laserCloud_sub = nh.subscribe("/multisense_sl/camera/points2",10, &CornerDetector::laserCloudCB, this);
}

bool CornerDetector::getPose(int index, geometry_msgs::Point &pixelCoordinates){
    bool poseXYZDetected = false;
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::StampedTransform stampedTransform;
    tf::Quaternion orientation;
    pcl::PointXYZRGB pcl_point;
   	StereoPointCloudColor::Ptr organized_cloud(new StereoPointCloudColor);
    geometry_msgs::Point msg;

    try
    {
        listener.waitForTransform("/world", "/left_camera_optical_frame", ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    // Obtaining a stereo point cloud for Z position and RGB values

    generateOrganizedCloud(disparityImageVect[0],colorImageVect[0],qMatrix,organized_cloud);

    pcl_point = organized_cloud->at(pixelCoordinates.x, pixelCoordinates.y);

    geometry_msgs::PointStamped corner_coordinates;
    corner_coordinates.header.frame_id= "left_camera_optical_frame";
    corner_coordinates.header.stamp = ros::Time::now();
    corner_coordinates.point.x = pcl_point.x;
    corner_coordinates.point.y = pcl_point.y;
    corner_coordinates.point.z = pcl_point.z;
    ROS_INFO("Camera Coordinates: %0.2f,%0.2f,%0.2f",pcl_point.x,pcl_point.y,pcl_point.z);
    geometry_msgs::PointStamped corner_coordinates_head;

    try{

        listener.transformPoint("/world",corner_coordinates, corner_coordinates_head);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    transform.setOrigin( tf::Vector3(corner_coordinates_head.point.x, corner_coordinates_head.point.y, corner_coordinates_head.point.z) );
    orientation.setRPY(0, 0, 0);
    transform.setRotation(orientation);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Corner_frame")); //Co-ordinates wrt left_camera_optical_frame

    msg.x = corner_coordinates_head.point.x;
    msg.y = corner_coordinates_head.point.y;
    msg.z = corner_coordinates_head.point.z;

    if (!((msg.x==0)&&(msg.y==0)&&(msg.z==0))){
    	poseXYZDetected = true;
    }

    //    ROS_INFO("Updating Message at index :%d",index );
    outputMessages[index]=msg;

    ROS_INFO("Ground Truth values:%d x: %0.2f, y: %0.2f, z: %0.2f",index+1, GroundTruth[index].x, GroundTruth[index].y, GroundTruth[index].z);
    ROS_INFO("Final coordinates index:%d x: %0.2f, y: %0.2f, z: %0.2f",index+1,outputMessages[index].x,outputMessages[index].y,outputMessages[index].z);
    ROS_WARN("ERROR Value:  index:= %d x: %0.2f, y: %0.2f, z: %0.2f ",index+1, GroundTruth[index].x - outputMessages[index].x ,GroundTruth[index].y - outputMessages[index].y ,GroundTruth[index].z - outputMessages[index].z);
    return poseXYZDetected;

}

void CornerDetector::imageCB(const sensor_msgs::ImageConstPtr& img) {

   cv_bridge::CvImagePtr inMsgPtr;
   geometry_msgs::Point pixelCoordinates;
   cv::Mat image;
   try
    {
        inMsgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge_exception: %s",e.what());
        return;
    }

	image=inMsgPtr->image.clone();
	colorImageVect.push_back(image);

    cv::Mat src_gray;

    cv::cvtColor(inMsgPtr->image,src_gray,CV_BGR2GRAY);

 	cv::Size patternsize(8,6);

 	if (once){
	bool patternfound = findChessboardCorners(inMsgPtr->image,patternsize,corners);

	drawChessboardCorners(inMsgPtr->image, patternsize, cv::Mat(corners), patternfound);

	// cv::imshow(OPENCV_WINDOW_1, inMsgPtr->image);
	// ROS_WARN("Corners :%d",corners.size());

	for(int flag =0;flag <48;flag++){
		pixelCoordinates.x = corners[flag].x;
        pixelCoordinates.y = corners[flag].y;
        // std::cout<<"Corners Detected at x: "<<pixelCoordinates.x<<" ,y: "<<pixelCoordinates.y<<std::endl;
        cornersDetected = true;
    }
    once = 0;
    }

	// cv::waitKey(0);
	// ROS_INFO("Got image coordinates");
}

void CornerDetector::dispCB(stereo_msgs::DisparityImageConstPtr msg){
	std_msgs::Header disp_header;
	cv::Mat disparity_;
	bool new_disp;
	uint8_t depth=sensor_msgs::image_encodings::bitDepth(msg->image.encoding);  //the size of the disparity data can be 16 or 32
	if (depth == 32)
	{
	    cv::Mat_<float> disparity(msg->image.height, msg->image.width,
	                              const_cast<float*>(reinterpret_cast<const float*>(&msg->image.data[0])));

	    disparity_=disparity.clone();
	    disp_header=msg->image.header;
	}
	else if(depth == 16)
	{
	    cv::Mat_<uint16_t> disparityOrigP(msg->image.height, msg->image.width,
	                                      const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&msg->image.data[0])));
	    cv::Mat_<float>   disparity(msg->image.height, msg->image.width);
	    disparity_ = disparityOrigP / 16.0f;
	    disparity_=disparity.clone();
	    disp_header=msg->image.header;
	}
	disparityImageVect.push_back(disparity_);
	if(disparityImageVect.size() != 0){
	read_dispImg = true;
	}
	// ROS_INFO("Got disparity image");

}

void CornerDetector::laserCloudCB(const sensor_msgs::PointCloud2 &msg){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*m_cloud);
}


void CornerDetector::generateOrganizedCloud( const cv::Mat &dispImage,
												   const cv::Mat &colorImage,
												   const cv::Mat Qmat,
												   StereoPointCloudColor::Ptr &cloud){
	cv::Mat xyz;
	int width=dispImage.cols;
	int height=dispImage.rows;


	cloud->height=height;
	cloud->width=width;
	cloud->resize(width*height);
	cv::reprojectImageTo3D(dispImage, xyz, Qmat, false);
	for(int u=0;u<dispImage.rows;u++)
		for(int v=0;v<dispImage.cols;v++)
		{
			if(dispImage.at<float>(cv::Point(v,u))==0.0)
				continue;
			cv::Vec3f cv_pt=xyz.at<cv::Vec3f>(cv::Point(v,u));
			pcl::PointXYZRGB pt;
			pt.x=cv_pt.val[0];
			pt.y=cv_pt.val[1];
			pt.z=cv_pt.val[2];
			cloud->at(v,u)=pt;

		}

}
void CornerDetector::calculatePose()
{
	geometry_msgs::Point pixelCoordinates;
	for (int j=0;j<=48;j++){
		pixelCoordinates.x = corners[j].x;
		pixelCoordinates.y = corners[j].y;
		bool poseFound = getPose(j,pixelCoordinates);
	}
}
void CornerDetector::calculateGroundTruth()
{
  double origin_x = 1.0;
  double origin_y = -0.6;
  double origin_z = 0.83;
  double square_size = 0.108;
  int rows = 8;
  int columns = 6;
  double cornersGt_y[6];
  double cornersGt_z[8];

  for(int i = 0; i < columns;  i++){
    cornersGt_y[i] = (origin_y + (square_size/2) + i*square_size);
  }
  for(int j = 0; j < rows;  j++){
    cornersGt_z[j] = (origin_z + (square_size/2) + j*square_size);
  }
  double cornersGroundTruth[48][3];


  for(int i = 0, k = 0, l = 5 ; i < (rows)*(columns) ; i++){
      cornersGroundTruth[i][0] = origin_x;
      cornersGroundTruth[i][1] = cornersGt_y[l];
      cornersGroundTruth[i][2] = cornersGt_z[k];
      k = k + 1;
      if(k > 7){
        k = 0;
        l = l - 1;
      }
  }

for(int i = 0; i < (rows-1)*(columns-1) ; i++){

    GroundTruth[i].x = cornersGroundTruth[i][0];
    GroundTruth[i].y = cornersGroundTruth[i][1];
    GroundTruth[i].z = cornersGroundTruth[i][2];
}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Corner_Detector");
	ros::NodeHandle nh_;
	ros::Rate loopRate = 10;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	m_cloud = temp_cloud;
	double fy = 476.7030836014194;
  double fx = 476.7030836014194;
  double cx = 400.5;
  double cy = 400.5;
  double tx = -0.07;

  //populate the qMatrix. it wont change with every image so no point in calculatin it everytime
  qMatrix=cv::Mat_<double>(4,4,0.0);
  qMatrix(0,0) =  fy*tx;
  qMatrix(1,1) =  fx*tx;
  qMatrix(0,3) = -fy*cx*tx;
  qMatrix(1,3) = -fx*cy*tx;
  qMatrix(2,3) =  fx*fy*tx;
  qMatrix(3,2) = -fy;
  qMatrix(3,3) =  0.0f;
	CornerDetector detect(nh_);
	detect.calculateGroundTruth();
  int done = 1;
	while(ros::ok()){
		// ROS_INFO("out %d , %d",read_dispImg,cornersDetected);
		if (read_dispImg && cornersDetected && done){
			for(int i = 0; i< corners.size(); i++)
			{
				ROS_INFO("Going calculate pose");
				detect.calculatePose();
			}
      break;
      done =0;
		}

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
