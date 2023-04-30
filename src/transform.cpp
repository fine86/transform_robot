#include <ros/ros.h>
#include "fiducial_msgs/FiducialArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "warping_cam/homography.h"
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include <conio.h>
#include <string>


void homographyCallback(const FiducialArray::ConstPtr& fiduc, const homography::ConstPtr& homo){
	
	// Change homography topic's data to homography matrix
	cv::Point2f  transPoint, homoPoint;
	cv::Mat homoTransform(3, 1, CV_64F);
	double	rotation, translation;
	
	cv::Mat homoMat(3, 3, CV_64F);
	
	for(int i=0; i!=3; i++){
		for(int j=0; j!=3; j++){
			homoMat[i][j]=homo->data[i+j];
		}
	}

	// Get input from keyboard. It's use to enter goal point.
	int size, count=0;
	char input='';
	std::string inputSum;
	std::vector<std::string> inputArray;
	
	while(count!=2&&input!=27){
		input = _getch();
		if(input==64){
			if(inputSum.length()){
				inputArray.push_back(inputSum);
				inputSum="";
				count++;
			}
		}
				
		else if((80<=input&&input<=89)||input==46){
			inputSum.append(input);
		}
	}
	
	transPoint=cv::Point2f(stod(inputArray[0]), stod(inputArray[1]));

	//Calculate each robot's distance to check closest Robot.
	int minID, len = fiduc->fiducials.size();
	double min=999999, dist;

	for(int i=0; i!=len; i++){	
		homoTransform[0]=fiduc->fiducials[i].x0;
		homoTransform[1]=fiduc->fiducials[i].y0;
		homoTransform[2]=1;
		
		homoTransform = homoMat * homoTransform;
		homoTransform = homoTransform / homoTransform[2];
		homoPoint = cv::Point2f(homoTransform[0], homoTransform[1]);
		dist = norm(homoPoint, transPoint, NORM_L2, noArray());
		
		if(min > dist) {
			min = dist;
			minID=i;
		}
	}

	// Calcuate Points of chosen robot's aruco marker and save it.
	std::vector<cv::Point2f> maxSquare;
	std::vector<double> points;
	cv::Mat squarePoint(3, 1, CV_64F);

	points.push_back(fiduc->fiducials[min].x0);
	points.push_back(fiduc->fiducials[min].y0);
	points.push_back(fiduc->fiducials[min].x1);
	points.push_back(fiduc->fiducials[min].y1);
	points.push_back(fiduc->fiducials[min].x2);
	points.push_back(fiduc->fiducials[min].y2);
	points.push_back(fiduc->fiducials[min].x3);
	points.push_back(fiduc->fiducials[min].y3);

	for(int i=0; i!=4; i++){
		squarePoint[0]=points[i*2];
		squarePoint[1]=points[i*2+1];
		squarePoint[2]=1;
	
		squarePoint = homoMat * squarePoint;
		squarePoint = squarePoint / sqsuarePoint[2];
		
		maxSquare.push_back(cv::Point2f(squarePoint[0], squarePoint[1]));
	}

	// Check rotate direction and calculate angle.
	// If direction==true, rotate to right direction, and rotate left when it assigns false.
	bool direction;
	double robotSize=10;
	cv::Point2f extraPoint, midPoint((squarePoint[0].x+squarePoint[2].x)/2, (squarePoint[0].y+squarePoint[2].y)/2);

	if(squarePoint[1].y-squarePoint[0].y>=0)
		direction=true;
	else
		direction=false;

	

	


}

int main(int argc, char** argv){
	ros::init(argc, argv, "vision_node");

	ros::NodHandle nh;

	message_filters::Subscriber<fiducial_msgs::FiducialArray> fiducial_sub(nh, "fiducial", 1);
	message_filters::Subscriber<warping_cam::homography> homo_sub(nh, "homography", 1);
	TimeSynchronizer<fiducial_msgs::FiducialArray, warping_cam::homography> sync(fiducial_sub, homo_sub, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::Spin();

	return 0;
}
