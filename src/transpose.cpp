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
		

		//Click goal point from mouse pointer.(Left Click)
		static cv::Point2f transPoint;
		cv::namedWindow("bird-eye VIEW");
		
		if(!(nh.getParam("/transform_robot/transPoint_x", transPoint.x) && nh.getParam("/transform_robot/transPoint_y", transPoint.y))){
			cv::setMouseCallback("bird-eye VIEW", mousePoint, static_cast<void*>(&transPoint));
			while(cv::waitKey(1) != 27){
				cv::imshow("bird-eye VIEW", img);
			}

			cv::destroyWindow("bird-eye VIEW");
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
		
		// Check position of goal and turtlebot, and calculate distance between them.
		cv::Point2f midPoint((maxSquare[0].x+maxSquare[2].x)/2, (maxSquare[0].y+maxSquare[2].y)/2);
		distance = calcDistance(transPoint, midPoint);
		ROS_INFO("Remaining distance to the goal is %fmm.", distance);

		//Draw trejactory of turtlebot and show it.
		cv::namedWindow("Robot Trajectory");
		cv::line(img, midPoint, transPoint, cv::Scalar(255, 0, 0), 3, 8, 0);
		cv::circle(img, transPoint, 7, cv::Scalar(0, 0, 255), -1);
		cv::circle(img, midPoint, 7, cv::Scalar(0, 255, 0), -1);
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
