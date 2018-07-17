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

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation(double desired_angle_radians);

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
	ros::Rate loop_rate(1000);
	ROS_INFO_STREAM("\n\n\n*****START MOVING*****\n");
	speed = 1;
	distance = 2;
	isForward = true;

	move(speed, distance, isForward);

	angular_speed = 10.0;
	angle = 90.0;
	clockwise = false;
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);

	move(speed, distance, isForward);
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	move(speed, distance, isForward);
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	move(speed, distance, isForward);
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	*/

	
	ros::Rate loop(100);
	turtlesim::Pose goal_pose;
	
	//ros::spinOnce();
	//ROS_INFO_STREAM("x: " << turtlesim_pose.x);

	/*goal_pose.x = turtlesim_pose.x;
	goal_pose.y = turtlesim_pose.y;
	goal_pose.theta = 0;
	moveGoal(goal_pose, 0.01);
	loop.sleep();
	*/
	
	goal_pose.x = turtlesim_pose.x + 1;
	goal_pose.y = turtlesim_pose.y + 1;
	goal_pose.theta = 0;
	moveGoal(goal_pose, 0.01);
	loop.sleep();
	
	setDesiredOrientation(0);
	loop.sleep();
}

void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;
	double Kv = 1.0;
	if(isForward)
	{
		vel_msg.linear.x = Kv * abs(speed);
	}
	else
	{
		vel_msg.linear.x = Kv * -abs(speed);
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

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;
	double Kw = 1.0;
	
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if(clockwise)
	{
		vel_msg.angular.z = Kw * -abs(angular_speed);
	}
	else
	{
		vel_msg.angular.z = Kw * abs(angular_speed);
	}

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	double t1;
	ros::Rate loop_rate(100);

	do {
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_angle < relative_angle);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees * PI / 180.0;
}

void setDesiredOrientation(double desired_angle_radians)
{
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians < 0) ? true : false);
	rotate(degrees2radians(180), abs(relative_angle_radians), clockwise);
}

//void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance)
void moveGoal(double relative_x, double relative_y, double distance_tolerance
{
	turtlesim::Pose goal_pose;
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;

	goal_pose.x = turtlesim_pose.x + relative_x;
	goal_pose.y = turtlesim_pose.y + relative_y;

	do {
		double Kv = 1.0;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

		vel_msg.linear.x = (Kv * e);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		double Kw = 4.0;

		vel_msg.angular.z = Kw * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		ROS_INFO_STREAM("x: " << turtlesim_pose.x);
		ROS_INFO_STREAM("y: " << turtlesim_pose.y);
		ROS_INFO_STREAM("theta: " << turtlesim_pose.theta);
		ROS_INFO_STREAM("goal x: " << goal_pose.x);
		ROS_INFO_STREAM("goal y: " << goal_pose.y);
		ROS_INFO_STREAM("goal theta: " << goal_pose.theta);
		loop_rate.sleep();
	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);

	std::cout << "end move goal" << std::endl;
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
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
