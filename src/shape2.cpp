#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"
#include <sstream>
#include <stdlib.h>
#include <math.h>
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
const double PI = 3.14159265359;

//void move(double speed, double distance, bool isForward);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double getDistance(double x1, double y1, double x2, double y2);

int main(int argc, char **argv)
{
	/*ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	
	//srand(time(0));

	ros::Rate rate(1);

	while(ros::ok())
	{
		geometry_msgs::Twist msg;
		//msg.linear.x = double(rand()) / double(RAND_MAX);
		//msg.angular.z = 2 * double(rand()) / double(RAND_MAX) - 1;
		msg.linear.x = 5.0;
		msg.angular.z = atan2(sqrt(3), -1);
		pub.publish(msg);

		//ROS_INFO_STREAM("Sending_random_velocity_command: " << " linear= "
		//		<< msg.linear.x << " angular= " << msg.angular.z);
		rate.sleep();
	}*/
	
	
	ros::init(argc, argv, "shape1");
	ros::NodeHandle nh;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	
	velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = nh.subscribe("/turtle1/pose", 10, poseCallback);
	
	/*
	ros::Rate loop_rate(10);
	ROS_INFO_STREAM("\n\n\n*****START MOVING*****\n");
	speed = 1;
	distance = 2;
	isForward = true;

	move(speed, distance, isForward);
	*/

	ros::Rate loop(100);
	turtlesim::Pose goal_pose;
	goal_pose.x = 5;
	goal_pose.y = 5;
	goal_pose.theta = 0;
	moveGoal(goal_pose, 0.01);
	loop.sleep();
}

/*
void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;
	if(isForward)
	{
		vel_msg.linear.x = abs(speed);
	}
	else
	{
		vel_msg.linear.x = -abs(speed);
	}

	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	double t1;

	do {
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance < distance);

	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}
*/

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance)
{
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;

	do {
		double Kv = 1.0;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

		vel_msg.linear.x = (Kv * e);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		double Kw = 1.5;

		vel_msg.angular.z = Kw * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);

	cout << "end move goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
