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
int once =1;
CornerDetector::CornerDetector(ros::NodeHandle nh) {
    m_image_sub = nh.subscribe("/multisense_sl/camera/left/image_rect_color", 1, &CornerDetector::imageCB, this);
    m_dispimg_sub = nh.subscribe("/multisense_sl/camera/disparity",1,&CornerDetector::dispCB,this);
    m_laserCloud_sub = nh.subscribe("/multisense_sl/camera/points2",1, &CornerDetector::laserCloudCB, this);
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
    ROS_INFO("Final coordinates x: %0.2f, y: %0.2f, z: %0.2f",outputMessages[index].x,outputMessages[index].y,outputMessages[index].z);
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
 	// cv::vector<cv::Point2f> corners;
 	if (once){
	bool patternfound = findChessboardCorners(inMsgPtr->image,patternsize,corners);

	drawChessboardCorners(inMsgPtr->image, patternsize, cv::Mat(corners), patternfound);

	cv::imshow(OPENCV_WINDOW_1, inMsgPtr->image);
	ROS_WARN("Corners :%d",corners.size());
	
	for(int flag =0;flag <48;flag++){
		pixelCoordinates.x = corners[flag].x;
        pixelCoordinates.y = corners[flag].y;
        std::cout<<"Corners Detected at x: "<<pixelCoordinates.x<<" ,y: "<<pixelCoordinates.y<<std::endl;
        cornersDetected = true;
    }
    once = 0;
    }
 
	cv::waitKey(3);
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
	
	
	ROS_INFO("w:%d h:%d",width,height);	
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
	for (int i = 0;i<corners.size();i++){
		pixelCoordinates.x = corners[i].x;
		pixelCoordinates.y = corners[i].y;
		bool poseFound = getPose(flag,pixelCoordinates);
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
	while(ros::ok()){
		ROS_INFO("out %d , %d",read_dispImg,cornersDetected);
		if (read_dispImg && cornersDetected){
			for(int i = 0; i<  corners.size(); i++)
			{
				ROS_INFO("Going calculate pose");
				detect.calculatePose();
			}
		}
		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
