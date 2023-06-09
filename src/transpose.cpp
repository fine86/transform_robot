#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "fiducial_msgs/FiducialArray.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <string>
#include <cmath>



class transformAndTranspose{
public:

	transformAndTranspose():nh(){
		pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		sub1 = nh.subscribe("/homography", 1, &transformAndTranspose::homographyCallback, this);
		
		sub2 = nh.subscribe("/fiducial_vertices", 1, &transformAndTranspose::transposeCallback, this);
	}

	static double calcDistance(cv::Point2f a, cv::Point2f b){
		double x = a.x - b.x;
		double y = a.y - b.y;
		double result = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
		return result;
	}

	static void mousePoint(int event, int x, int y, int flags, void *param){
		if(event == cv::EVENT_LBUTTONDOWN){
			cv::Point2f *transPoint = (cv::Point2f*)param;
			transPoint->x = double(x);
			transPoint->y = double(y);
			ROS_INFO("Transport turtlebot to (%d, %d)", x, y);
		}
	}

	cv::Point2f calcMid(std::vector<cv::Point2f> square){
		double a1, a2, b1, b2, x, y;
		
		a1 = (square[2].y-square[0].y)/(square[2].x-square[0].x);
		a2 = (square[3].y-square[1].y)/(square[3].x-square[1].x);
		
		b1 = square[0].y - a1 * square[0].x;
		b2 = square[1].y - a2 * square[1].x;

		x = (b2-b1)/(a1-a2);
		y = x * a1 + b1;

		cv::Point2f result(x, y);

		return result;
	}

	void homographyCallback(const sensor_msgs::ImageConstPtr& homo){
		// Save transformed image.
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(homo, sensor_msgs::image_encodings::BGR8);

		img = cv_ptr->image;
	}


	void transposeCallback(const fiducial_msgs::FiducialArray::ConstPtr& fiduc){
		// Check package received non-empty fiducial message.
		ros::Time start_time;
		ros::Rate loop_rate(10);
		int delay=1;
		
		if(fiduc->fiducials.empty()){
			if(!nh.getParam("/transform_robot/delay", delay))
				nh.setParam("transform_robot/delay", delay);
			else{
				if(delay > 100){
					ROS_INFO("No turtlebot detected. System shutdown!");
					nh.deleteParam("/transform_robot/transPoint_x");
					nh.deleteParam("/transform_robot/transPoint_y");
					ros::shutdown();
				}
				else if(delay > 30){
					ang = 0.1;
					msg.angular.z = ang; 			
					start_time = ros::Time::now();
					while(ros::Time::now() - start_time < ros::Duration(1)){
						pub.publish(msg);
						loop_rate.sleep();
					}
					
					msg.angular.z = 0;
					start_time = ros::Time::now();
					while(ros::Time::now() - start_time < ros::Duration(1)){
						pub.publish(msg);
						loop_rate.sleep();
					}
				}
				
				delay++;
				nh.setParam("transform_robot/delay", delay);
				
			}
			return;
		}

		if(nh.getParam("/transform_robot/delay", delay))
			nh.deleteParam("/transform_robot/delay");
		
		// receive intrinsic, extrinsic params
		std::vector<double> intrinsicArray, poseArray, RArray, homoArray;
		
		nh.getParam("usb_cam/camera_matrix/data", intrinsicArray);
		nh.getParam("usb_cam/extrinsic/pose", poseArray);
		nh.getParam("usb_cam/extrinsic/R", RArray);
		nh.getParam("homography/data", homoArray);

		cv::Mat intrinsic(3, 3, CV_64F);
		for(int i=0; i!=3; i++){
			for(int j=0; j!=3; j++){
				intrinsic.at<double>(i, j) = intrinsicArray[i * 3 + j];
			}
		}
		
		cv::Mat pose(3, 1, CV_64F);	
		for(int i=0; i!=3; i++){
			pose.at<double>(i, 0) = poseArray[i];
		}
		
		cv::Mat R(3, 3, CV_64F);
		for(int i=0; i!=3; i++){
			for(int j=0; j!=3; j++){
				R.at<double>(i, j) = RArray[i * 3 + j];
			}
		}


		cv::Mat homography(3, 3, CV_64F);
		for(int i=0; i!=3; i++){
			for(int j=0; j!=3; j++){
				homography.at<double>(i, j) = homoArray[i*3+j];
			}
		}
		
		//define inverse matrix
		cv::Mat invHomography;
		cv::Mat R_inv;
		cv::Mat invIntrinsic;
		
		cv::invert(homography, invHomography);
		cv::invert(R, R_inv);
		cv::invert(intrinsic, invIntrinsic);

		cv::Mat calculating1, calculating2;
		double S;

		//Click goal point from mouse pointer.(Left Click)
		static cv::Point2f transPoint;
		cv::namedWindow("bird-eye VIEW");
		
		if(!(nh.getParam("/transform_robot/transPoint_x", transPoint.x) && nh.getParam("/transform_robot/transPoint_y", transPoint.y))){
			cv::setMouseCallback("bird-eye VIEW", mousePoint, static_cast<void*>(&transPoint));
			while(cv::waitKey(1) != 27){
				cv::imshow("bird-eye VIEW", img);
			}

			cv::destroyWindow("bird-eye VIEW");
			cv::Mat transform(3, 1, CV_64F);
			transform.at<double>(0) = transPoint.x;
			transform.at<double>(1) = transPoint.y;
			transform.at<double>(2) = 1.0;

			cv::Mat cameraTarget = invHomography * transform;
			calculating1 = invIntrinsic * cameraTarget;
	       		calculating2 = R_inv * calculating1;

			S = pose.at<double>(2)/calculating2.at<double>(2);

			cv::Mat transWorldPoint = pose -  calculating2 * S;

			transPoint.x = transWorldPoint.at<double>(0);
			transPoint.y = transWorldPoint.at<double>(1);
			
			nh.setParam("/transform_robot/transPoint_x", transPoint.x);
			nh.setParam("/transform_robot/transPoint_y", transPoint.y);
		}

		//Calculate each robot's distance to check closest Robot.
		int minID;
		double min=999999, dist;
		cv::Point2f homoPoint;
		
		for(const auto& fiducial : fiduc->fiducials){
			homoPoint = cv::Point2f(fiducial.x0, fiducial.y0);
			dist = calcDistance(homoPoint, transPoint);

			if(min > dist){
				min = dist;
				minID = fiducial.fiducial_id;
			}
		}

		// Calcuate Points of chosen robot's aruco marker and save it.
		std::vector<cv::Point2f> maxSquare;
		std::vector<double> points;
		
		// fiducial points
		// 0---------------1
		// |               |
		// |               |
		// |               |
		// |               |
		// |               |
		// |               |
		// 3---------------2

		for(const auto& fiducial : fiduc->fiducials){
			if(fiducial.fiducial_id == minID){
				points.push_back(fiducial.x0);
				points.push_back(fiducial.y0);
				points.push_back(fiducial.x1);
				points.push_back(fiducial.y1);
				points.push_back(fiducial.x2);
				points.push_back(fiducial.y2);
				points.push_back(fiducial.x3);
				points.push_back(fiducial.y3);
			}
		}		

		for(int i=0; i!=4; i++)
			maxSquare.push_back(cv::Point2f(points[i*2], points[i*2+1]));
		// Check position of goal and turtlebot, transform these into world coordinate and calculate distance between them.
		cv::Point2f cameraMidPoint= calcMid(maxSquare);
		cv::Point2f midPoint((maxSquare[0].x+maxSquare[2].x)/2, (maxSquare[0].y+maxSquare[2].y)/2);



		//calculate world coordinate
		cv::Mat imgPoint(3, 1, CV_64F);
		imgPoint.at<double>(0) = cameraMidPoint.x;
		imgPoint.at<double>(1) = cameraMidPoint.y;
		imgPoint.at<double>(2) = 1;


		std::cout << "midPoint" << std::endl;
		std::cout << cameraMidPoint.x << std::endl;
		std::cout << cameraMidPoint.y << std::endl;


		calculating1 = invIntrinsic * imgPoint;
	       	calculating2 = R_inv * calculating1;

		S = (pose.at<double>(2)+192)/calculating2.at<double>(2);

		cv::Mat turtlebotWorldPoint = pose - calculating2 * S;

		cv::Point2f turtlebotPoint(turtlebotWorldPoint.at<double>(0), turtlebotWorldPoint.at<double>(1));
	
		// transform target point as real world coordinate and calculate distance	
		
		
		

		ROS_INFO("Turtlebot Position : (%f, %f)", turtlebotPoint.x, turtlebotPoint.y);	
		ROS_INFO("Target Position : (%f, %f)", transPoint.x, transPoint.y);	
		
		distance = calcDistance(transPoint, turtlebotPoint);
		
		ROS_INFO("Remaining distance to the goal is %fmm.", distance);

		//Draw trejactory of turtlebot and show it.
		cv::namedWindow("Robot Trajectory");
		cv::line(img, turtlebotPoint, transPoint, cv::Scalar(255, 0, 0), 3, 8, 0);
		cv::circle(img, transPoint, 7, cv::Scalar(0, 0, 255), -1);
		cv::circle(img, turtlebotPoint, 7, cv::Scalar(0, 255, 0), -1);
		while(cv::waitKey(1) != 27){
			cv::imshow("Robot Trajectory", img);
		}
	
		// If distance under 0.01m, finish node
		if(distance * env_size / 1000 <= 10){
			nh.deleteParam("/transform_robot/transPoint_x");
			nh.deleteParam("/transform_robot/transPoint_y");

			ROS_INFO("Arrived at the goal! System Finished");
			ros::shutdown();
		}


		// Check rotate direction and calculate angle.
		double angle1, angle2;
		cv::Point2f frontPoint((maxSquare[0].x+maxSquare[1].x)/2, (maxSquare[0].y+maxSquare[1].y)/2);

		angle1 = std::atan2(frontPoint.y-midPoint.y, frontPoint.x-midPoint.x);
		angle2 = std::atan2(transPoint.y-midPoint.y, transPoint.x-midPoint.x);
		
		direction = angle1 - angle2;

		// Publish Twist message to turtlebot.
		if(direction < 0)
			ang = -0.3;
		else
			ang = 0.3;
		
		time1 = direction / ang;
		time2 = (distance * env_size) / (lin * 1000 * 1000);
		
		msg.angular.z = ang;
		start_time = ros::Time::now();
		while(ros::Time::now() - start_time < ros::Duration(time1)){
			pub.publish(msg);
			loop_rate.sleep();
		}


		msg.angular.z=0;
		start_time = ros::Time::now();
		while(ros::Time::now() - start_time < ros::Duration(time3)){
			pub.publish(msg);
			loop_rate.sleep();
		}


		msg.linear.x=lin;
		start_time = ros::Time::now();

		while(ros::Time::now() - start_time < ros::Duration(time2)){
			pub.publish(msg);
			loop_rate.sleep();
		}

		msg.linear.x=0;
		start_time = ros::Time::now();
		
		while(ros::Time::now() - start_time < ros::Duration(time3)){
			pub.publish(msg);
			loop_rate.sleep();
		}

		cv::destroyAllWindows();
	}



private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub1;
	ros::Subscriber sub2;

	// Twist
	// linear : 0.05 m/s
	// angular : 0.5 rad/s
	geometry_msgs::Twist msg;
	double time1, time2, time3=1, ang, lin=0.05;
	double direction, distance;
	int env_size=800;
	
	cv::Mat img;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Twist");

  transformAndTranspose TATObject;

  ros::spin();

  return 0;
}
