#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
    // 初始化ROS节点，命名为"talker_node"
    ros::init(argc, argv, "talker_node");
    
    // 创建节点句柄
    ros::NodeHandle n;
    
    // 创建一个Publisher，发布名为"chatter"的话题，消息类型为std_msgs::String，队列长度1000
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    // 设置循环频率为10Hz
    ros::Rate loop_rate(10);
    
    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello ROS! " << count; // 包含实验要求的 "Hello ROS!"
        msg.data = ss.str();
        
        // 打印将要发布的消息到终端
        ROS_INFO("%s", msg.data.c_str());
        
        // 发布消息
        chatter_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep(); // 休眠以保证10Hz的频率
        ++count;
    }
    return 0;
}
