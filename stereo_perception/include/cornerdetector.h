#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class CornerDetector
{

private :
		/**
   		* @brief m_imageSub ros subscriber that subscribes to the image topic
   		*/

		ros::Subscriber m_image_sub;
		/**
		* @brief imageCB Callback function to detect corners of the chessboard when a new image is received on the subscribed topic
		* @param img constant pointer to the image message
		*/

		void imageCB(const sensor_msgs::ImageConstPtr& msg);
public:
  /**
   * @brief LightDetect Constructor of LightDetect class.
   * @param n ros nodehandle
   */
  CornerDetector(ros::NodeHandle nh);



};

#endif