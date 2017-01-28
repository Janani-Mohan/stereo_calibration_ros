#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

typedef	pcl::PointXYZRGB					StereoPointColor;
typedef pcl::PointCloud<StereoPointColor> 	StereoPointCloudColor;

class CornerDetector
{

private :
		/**
   		* @brief m_imageSub ros subscriber that subscribes to the image topic
   		*/
		ros::Subscriber m_image_sub;
		ros::Subscriber m_dispimg_sub;
		ros::Subscriber m_laserCloud_sub;
		/**
		* @brief imageCB Callback function to detect corners of the chessboard when a new image is received on the subscribed topic
		* @param img constant pointer to the image message
		*/

		void imageCB(const sensor_msgs::ImageConstPtr& msg);
		void dispCB(stereo_msgs::DisparityImageConstPtr msg);
		void generateOrganizedCloud(const cv::Mat &dispImage,
												   const cv::Mat &colorImage,
												   const cv::Mat Qmat,
												   StereoPointCloudColor::Ptr &cloud);
		void laserCloudCB(const sensor_msgs::PointCloud2 &msg);
		bool getPose(int index, geometry_msgs::Point &pixelCoordinates);
		

public:
  /**
   * @brief LightDetect Constructor of LightDetect class.
   * @param n ros nodehandle
   */
	CornerDetector(ros::NodeHandle nh);
	void calculatePose();
	void calculateGroundTruth();

};

#endif
