#include "ros/ros.h"
#include "std_msgs/String.h"

// 接收到订阅消息后的回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    // 初始化ROS节点，命名为"listener_node"
    ros::init(argc, argv, "listener_node");
    
    // 创建节点句柄
    ros::NodeHandle n;
    
    // 创建一个Subscriber，订阅名为"chatter"的话题，注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    
    // 循环等待回调函数
    ros::spin();
    
    return 0;
}
