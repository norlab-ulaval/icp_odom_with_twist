//
// Created by dominic on 2021-07-29.
//

#include <iostream>
#include <mutex>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

ros::Publisher mergedOdomPublisher;
nav_msgs::Odometry icpOdom;
std::mutex icpOdomLock;
sensor_msgs::Imu imuData;
std::mutex imuDataLock;

void icpOdomCallback(const nav_msgs::Odometry& odometry)
{
    icpOdomLock.lock();
    icpOdom = odometry;
    icpOdomLock.unlock();
}

void imuOdomCallback(const sensor_msgs::Imu& imu)
{
    imuDataLock.lock();
    imuData = imu;
    imuDataLock.unlock();
}

nav_msgs::Odometry mergeOdomMsgs(const nav_msgs::Odometry& icpOdometry, const sensor_msgs::Imu& imuData)
{
    nav_msgs::Odometry mergedOdometry;
    mergedOdometry = icpOdometry;
    mergedOdometry.twist.twist.angular = imuData.angular_velocity;
//    mergedOdometry.twist.twist.linear.x = 0.0;
//    mergedOdometry.twist.twist.linear.y = 0.0;
//    mergedOdometry.twist.twist.linear.z = 0.0;
    return mergedOdometry;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "norlab_icp_odom_with_twist_node");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");
    mergedOdomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("control_odom", 1000, true);
    ros::Subscriber icpOdomSubscriber = nodeHandle.subscribe("icp_odom", 1000, icpOdomCallback);
    ros::Subscriber imuOdomSubscriber = nodeHandle.subscribe("imu/data", 1000, imuOdomCallback);
    ros::Rate loopRate(20);
    while(nodeHandle.ok())
    {
        ros::spinOnce();
        mergedOdomPublisher.publish(mergeOdomMsgs(icpOdom, imuData));
        loopRate.sleep();
    }
    return 0;
}


