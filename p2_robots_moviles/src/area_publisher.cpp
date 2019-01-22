/*
***************************
***************************
*******  S E T U P  *******
***************************
***************************
*/

//Include dependencies
	#include <ros/ros.h>
	#include <image_transport/image_transport.h>
//	#include <opencv2/highgui/highgui.hpp>
	#include <cv_bridge/cv_bridge.h>
	#include <std_msgs/Float64MultiArray.h>

	double getArea(cv::Mat img);



/*
***************************
***************************
**** C A L L B A C K S ****
***************************
***************************
*/

std_msgs::Float64MultiArray areas;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat img_bgr, img_hsv; //Variables to store images in different color spaces
		cv::Mat red1, red2, green, blue;
		img_bgr = cv_bridge::toCvShare(msg, "bgr8")->image; //Store original image (in BGR)
		cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV); //Convert image from BGR to HSV
//		cv::imshow("view", img_bgr); //Show BRG image

		//red (160, 57, 58) ilum (120, 0, 0) ilumilum (154, 50, 50)/ HSV (360, 64, 63) ilum (0, 100, 47) ilumilum (0 68 60)
		//blue (1, 0, 102) ilum (58, 58, 160) / HSV (241, 100, 47) ilum (240 64 63)
		//green (0, 102 , 0) ilum (58, 160, 58) / HSV (120, 100, 47) ilum (120, 64, 63)
		//HSV openCV dividir por 2 la H y multiplicar por 2.5 S y V
		//Rojo (180, 160, 160), ilum (0, 255, 117.5), ilumilum (0, 160, 160)
		//Azul (120, 255, 117.5), ilum (120, 160, 160)
		//Verde (60, 255, 160), ilum (60, 160, 160)
		//Threshold red colors
		cv::inRange(img_hsv, cv::Scalar(0, 50, 30), cv::Scalar(25, 255, 255), red1);
		cv::inRange(img_hsv, cv::Scalar(145, 50, 30), cv::Scalar(179, 255, 255), red2);
		red1 = red1+red2; //Add the two pictures
//		cv::imshow("red", red1); //Show red colors
		areas.data[0] = getArea(red1);

		//Threshold green colors
		cv::inRange(img_hsv, cv::Scalar(25, 50, 30), cv::Scalar(75, 255, 255), green);
//		cv::imshow("green", green); //Show green colors
		areas.data[1] = getArea(green);

		//Threshold blue colors
		cv::inRange(img_hsv, cv::Scalar(75, 50, 30), cv::Scalar(140, 255, 255), blue);
//		cv::imshow("blue", blue); //Show blue colors
		areas.data[2] = getArea(blue);


		/*//Threshold red colors
		cv::inRange(img_bgr, cv::Scalar(0, 0, 100), cv::Scalar(70, 70, 255), red1);
		areas.data[0] = getArea(red1);
		//Threshold green colors
		cv::inRange(img_bgr, cv::Scalar(0, 80, 0), cv::Scalar(70, 200, 70), green);
		areas.data[1] = getArea(green);
		//Threshold green colors
		cv::inRange(img_bgr, cv::Scalar(90, 0, 0), cv::Scalar(255, 70, 70), blue);
		areas.data[2] = getArea(blue);*/

		//std::cout << areas.data[0] << "  " << areas.data[1] << "  " << areas.data[2] << "  " << std::endl;

//		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}



/*
***************************
***************************
***** M A I N   F N C *****
***************************
***************************
*/

int main(int argc, char **argv)
{
  //Initialize global variables
    areas.data.push_back(0.0);
    areas.data.push_back(0.0);
    areas.data.push_back(0.0);

  //Initialize OpenCV windows
//    cv::namedWindow("view"); cv::namedWindow("red"); cv::namedWindow("green"); cv::namedWindow("blue");
//    cv::startWindowThread();

  //Initialize ROS nodes
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    //Camera subscriber
	image_transport::ImageTransport it(nh);
	//image_transport::Subscriber sub = it.subscribe("/webcam/image_raw", 1, imageCallback); //Camera topic subscriber (pc)
	//image_transport::Subscriber sub = it.subscribe("/pepper_robot/naoqi_driver/camera/front/image_raw", 1, imageCallback); //Camera topic subscriber (pepper)
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback); //Camera topic subscriber (pc) (RGB astra)

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

//  cv::destroyWindow("view"); cv::destroyWindow("red"); cv::destroyWindow("green"); cv::destroyWindow("blue");
}



/*
***************************
***************************
***** A U X.  F N C S *****
***************************
***************************
*/

double getArea(cv::Mat img)
{
	//Get contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		//cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
		cv::findContours(img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
	//Get the biggest contour
		int maxContour = 0; //Store the biggest contour's index
		float maxArea = 0;
		float auxArea = 0;

		//if(contours.size() >= 1)
		//{
			for(int i=0; i<(contours.size()); i++)
			{
				auxArea=cv::contourArea(contours[i]);
				
				if(maxArea < auxArea)
				{
					maxArea = auxArea;
				}
			}
		//}

	//Get area    
		return maxArea;
}
