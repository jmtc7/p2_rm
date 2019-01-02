#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float64MultiArray areas;

double getArea(cv::Mat img)
{
	//Get contours
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	//Get the biggest contour
		int maxContour = 0; //Store the biggest contour's index
		float maxArea = 0;
		float auxArea = 0;

		if(contours.size() >= 1)
		{
			for(int i=0; i<(contours.size()-1); i++)
			{
				auxArea=cv::contourArea(contours[i]);
				
				if(maxArea < auxArea)
				{
					maxArea = auxArea;
				}
			}
		}

	//Get area    
		return maxArea;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat img_bgr, img_hsv; //Variables to store images in different color spaces
		cv::Mat red1, red2, green, blue;
		img_bgr = cv_bridge::toCvShare(msg, "bgr8")->image; //Store original image (in BGR)
		cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV); //Convert image from BGR to HSV
		cv::imshow("view", img_bgr); //Show BRG image

		//Threshold red colors
		cv::inRange(img_hsv, cv::Scalar(0, 128, 102), cv::Scalar(20, 255, 255), red1);
		cv::inRange(img_hsv, cv::Scalar(155, 128, 102), cv::Scalar(180, 255, 255), red2);
		red1 = red1+red2; //Add the two pictures
		cv::imshow("red", red1); //Show red colors
		areas.data[0] = getArea(red1);

		//Threshold green colors
		cv::inRange(img_hsv, cv::Scalar(35, 128, 102), cv::Scalar(75, 255, 255), green);
		cv::imshow("green", green); //Show green colors
		areas.data[1] = getArea(green);

		//Threshold blue colors
		cv::inRange(img_hsv, cv::Scalar(80, 128, 102), cv::Scalar(130, 255, 255), blue);
		cv::imshow("blue", blue); //Show blue colors
		areas.data[2] = getArea(blue);

		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
  //Initialize global variables
    areas.data.push_back(0.0);
    areas.data.push_back(0.0);
    areas.data.push_back(0.0);

  //Initialize OpenCV windows
    cv::namedWindow("view"); cv::namedWindow("red"); cv::namedWindow("green"); cv::namedWindow("blue");
    cv::startWindowThread();

  //Initialize ROS nodes
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    //Camera subscriber
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/webcam/image_raw", 1, imageCallback); //Camera topic subscriber (pc)
	//image_transport::Subscriber sub = it.subscribe("/pepper_robot/naoqi_driver/camera/front/image_raw", 1, imageCallback); //Camera topic subscriber (pepper)
    //image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback); //Camera topic subscriber (pc) (RGB astra)

    //Areas publishers
      ros::Publisher areas_publisher = nh.advertise<std_msgs::Float64MultiArray>("/rgb_areas", 1);

  //Publish areas
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
	areas_publisher.publish(areas); //Publish areas

    ros::spinOnce();
    loop_rate.sleep();
  }

  cv::destroyWindow("view"); cv::destroyWindow("red"); cv::destroyWindow("green"); cv::destroyWindow("blue");
}
