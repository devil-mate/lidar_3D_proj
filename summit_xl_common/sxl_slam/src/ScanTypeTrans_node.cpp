#include "ScanTypeTrans_node.h"
#include "sensor_msgs/LaserEcho.h"
ScanTypeTransNode::ScanTypeTransNode():nh("~"){
    initParams();
    init();
}
ScanTypeTransNode::~ScanTypeTransNode(){
    
}
void ScanTypeTransNode::initParams(){
    // subScanTopic_=scan;
    // nh.getParam("subScanTopic",subScanTopic_);
    nh.param<std::string>("subScanTopic",subScanTopic_,"scan");
    nh.param<std::string>("frame_id",frameId_,"laserScan_link");
}
void ScanTypeTransNode::init(){
    scanPub_ = nh.advertise<sensor_msgs::MultiEchoLaserScan>("out_scan",10);
    subVec_.push_back(n.subscribe<sensor_msgs::LaserScan>(subScanTopic_,5,
                                                &ScanTypeTransNode::laserScanCb,this)); 
}

void ScanTypeTransNode::laserScanCb(const sensor_msgs::LaserScan::ConstPtr& msg){

    //  判断订阅者是否连接
    if (scanPub_.getNumSubscribers() == 0) {
        return;
    }
    sensor_msgs::MultiEchoLaserScan pubMsg;
    pubMsg.header = msg->header;
    pubMsg.header.frame_id = frameId_;
    pubMsg.angle_min = msg->angle_min;
    pubMsg.angle_max = msg->angle_max;
    pubMsg.angle_increment = msg->angle_increment;
    pubMsg.time_increment = msg->time_increment;
    pubMsg.scan_time = msg->scan_time;
    pubMsg.range_min = msg->range_min;
    pubMsg.range_max = msg->range_max;
    
    for(uint32_t i = 0; i <msg->ranges.size(); i++) {
        sensor_msgs::LaserEcho laserEcho;
        // laserEcho.echoes[0]=msg->ranges[i];
        pubMsg.ranges.push_back(laserEcho);
    }
    scanPub_.publish(pubMsg);



}
int main(int argc,char **argv)
{
    std::string nodeName; 
    ros::init(argc,argv,"controlBox_node");
    nodeName = ros::this_node::getName();
    try{
        //节点重名，在节点名后添加随机数
//        ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName);
        ScanTypeTransNode scanTypeTransNode;
        // LOGINFO("%s start working",nodeName.c_str());
        ROS_INFO("%s start working",nodeName.c_str());
        ros::spin();
    }catch(...) {
        ROS_ERROR("[FATAL] %s exit",nodeName.c_str());
        //TODO 节点退出前资源释放
        ros::shutdown();
//        exit(0);
    }

    return 0;

}