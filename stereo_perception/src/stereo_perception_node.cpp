#include "cornerdetector.h"

static const std::string OPENCV_WINDOW_1 = "Corner Detection";
int flag = 0;
int i = 0;

CornerDetector::CornerDetector(ros::NodeHandle nh) {
    m_image_sub = nh.subscribe("/multisense_sl/camera/left/image_raw", 1, &CornerDetector::imageCB, this);
}

void CornerDetector::imageCB(const sensor_msgs::ImageConstPtr& img)
 {

   cv_bridge::CvImagePtr inMsgPtr;
   geometry_msgs::Point pixelCoordinates;

   try
    {
        inMsgPtr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8); 
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge_exception: %s",e.what());
        return;
    }

    cv::Mat src_gray;

    cv::cvtColor(inMsgPtr->image,src_gray,CV_BGR2GRAY);

 	cv::Size patternsize(8,6);
 	cv::vector<cv::Point2f> corners;

	bool patternfound = findChessboardCorners(inMsgPtr->image,patternsize,corners);

	drawChessboardCorners(inMsgPtr->image, patternsize, cv::Mat(corners), patternfound);

	cv::imshow(OPENCV_WINDOW_1, inMsgPtr->image);
	
	if (flag <= 48){
		pixelCoordinates.x = corners[flag].x;
        pixelCoordinates.y = corners[flag].y;
        std::cout<<"Corners Detected at x: "<<pixelCoordinates.x<<" ,y: "<<pixelCoordinates.y<<std::endl;
        flag ++;
    }

	cv::waitKey(3);
}

int main(int argc, char** argv)
{
 ros::init(argc, argv, "Corner_Detector");
 ros::NodeHandle nh_;
 CornerDetector detect(nh_);
 ros::spin();
 return 0;
}