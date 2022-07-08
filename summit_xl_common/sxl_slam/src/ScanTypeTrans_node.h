/**
 * @file ScanTypeTrans.h
 * @author your name (you@domain.com)
 * @brief  
 * @version 0.1
 * @date 2022-07-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef SCAN_TYPE_TRANS_NODE
#define SCAN_TYPE_TRANS_NODE

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>

class ScanTypeTransNode{
public:
    ScanTypeTransNode();
    ~ScanTypeTransNode();
private:
    ros::NodeHandle n;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subVec_;
    ros::Publisher scanPub_;
    std::string subScanTopic_;
    std::string frameId_;
    void init();
    void initParams();
    void laserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg);

};


#endif 

