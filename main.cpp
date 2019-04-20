//2019-04-08, I. Margolin, S. Diane

/*
Узел достовляющий робота в нужную точку

вход ros - координаты целевой точки или 
вход ros - данные с лидара
вход http - с потолочной навигации
вход ros - одометрия
вход ros - порог остановки
выход ros - управляющие воздействия для драйвера колес
выход ros - уточненная одометрия

*/

#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <time.h>


using namespace std;

ros::Subscriber odom_subscriber; //данные одометрии
ros::Subscriber laser_subscriber; //данные лазерного дальномера
ros::Publisher control_publisher; //управляющие воздействия


float xt=1, yt=1, enough_dist=0.2; //целевая точка

float x,y,a;//положение робота

float obs_angle, obs_dist; //угол на ближайшее препятствие относительно робота, расстояние до препятствия 

//ф-ция обратного вызова для приема одометрии
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	x=msg->pose.pose.position.x;
	y=msg->pose.pose.position.y;
	a=msg->twist.twist.angular.z;
}

int cntLaser=0;
//ф-ция обратного вызова для приема сканов с лидара
void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	cntLaser++;
	
	if(cntLaser%10==0) //чтоб отсылалось не очень часто
	{
		//msg->ranges - Массив Float32 (в Vector`e)
		
		// ищем минимальное расстояние до препятствия
		float min=0, imin=0;
		int step=3;//для скорости
		for(int i=0;i<msg->ranges.size();i+=step)
		{
			float x=msg->ranges[i];	
			if(!(msg->range_min <= msg->ranges[i] 
			&& msg->ranges[i] <= msg->range_max)) 
			x=msg->range_max; //Inf, NaN
			
			if(x<min) {imin=i; min=x;}
		}	
		obs_dist=min;
		float C2=(msg->ranges.size()-1)/2.0;
		//в левой части массива  угол положителен, т.к. ось Y направлена влево от юбота
		obs_angle = (C2 - imin)/C2; // от -1 до +1
		cntLaser=0;		
	}
}
void SendControl(float fwd, float left, float rotLeft)
{	
	geometry_msgs::Twist cmd_vel_msg;
	
	cmd_vel_msg.linear.x=fwd;
    cmd_vel_msg.linear.y=left;
    cmd_vel_msg.angular.z=rotLeft;
	
	control_publisher.publish(cmd_vel_msg);
}

//приводит угол в диапазон -pi;+pi
float limang(float a)
{
	const float pi = M_PI, pi2 = pi * 2;
	while (a > pi) a -= pi2;
	while (a < -pi) a += pi2;
	return a;
}
//обрезает значение
float limval(float a, float max)
{
	if(a<-max) return -max;
	if(a>max) return max;
	return a;
}
void ControlPlatform()
{
	//вектор от робота к цели
	float dx=xt-x;
	float dy=yt-y;
	
	//азимут на цель
	float at = atan2(dy,dx);	
	//доворот робота на цель
	float da=limang(at-a);

	float distToObs = sqrt(dx*dx + dy*dy);//вычисляем расстояние до объекта
	
	float k=0.1;//макс. скорость робота
	float vx=0.1;//limval(distToObs*k, k);
	float vy=0;//limval(dy, k);
	float va=limval(da, k);
	
	//TODO: пересчитать dxLocal dyLocal 
	
	if (distToObs <= enough_dist)//если робот доехал до точки то остановится
	{
		vx=0; vy=0; va=0;
	}
	else
	{
			cout << "Target azimuth global = " << at
			<< "; azimuth local = "<< da			
			<< "; a = "<< a
			<< "; x = "<< x
			<< "; y = "<< y
			<<endl;
	}
	//TODO: lidars
	//vy=0;//для движения без бокового проскальзывания
	SendControl(vx, vy, va);
}

int main(int argc, char** argv)
{
	cout << "youbot_move: Starting" << endl;
	
	ros::init(argc, argv, "youbot_move");
	ros::NodeHandle nh;
	
	string topic_odom= "/odom";
	
	string topic_laser="/scan";
	
	string topic_control="/cmd_vel";
	
	//Ros communications:
    control_publisher = nh.advertise<geometry_msgs::Twist>(topic_control, 1000);	 	
       
	odom_subscriber=nh.subscribe(topic_odom, 1000, &odomCallback);
	laser_subscriber=nh.subscribe(topic_laser, 1000, &LaserScanCallback);
	
	cout << "youbot_move: Callbacks Init OK" << endl;

    double hertz=20;

    ros::Rate loop_rate(hertz);
	
	int cnt=0;
		
    while(ros::ok())
    {
	    ros::spinOnce();
		loop_rate.sleep();
		if(cnt++<1) 
		cout << "youbot_move: First ROS iter OK" << endl;

		ControlPlatform();
    }
	cout <<"youbot_move: Finishing"<< endl;
	
	return 0;
}