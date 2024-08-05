//标准库文件
#include "ros/ros.h"
#include "std_msgs/String.h"

//标准的发布者对象实现
int main(int argc, char** argv)
{
  //初始化ROS节点
  ros::init(argc, argv, "robotarmClient");
  // Create robotarmController publisher
  //创建ROS节点句柄
  ros::NodeHandle lNodeHandler;
  //创建发布者
  ros::Publisher lRobotarmCommandPublisher = lNodeHandler.advertise<std_msgs::String>("robotarmCommand", 1000);
  // Publish commands
  if(ros::ok())
  {
    sleep(1);
    std_msgs::String msg;

    //发布动作所需电机消息
    //用于控制机械臂的每个关节（假设有六个关节，编号为0到5）移动到特定的位置
    //并在所有关节都达到目标位置后等待2000毫秒
    
    // Pick up and drop the cup to the right
    msg.data = "#0P1500#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1257#2P1500#3P1735#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(5);
    msg.data = "#0P1500#1P1130#2P1292#3P1684#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1130#2P1292#3P1684#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1595#1P1429#2P1515#3P1579#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1595#1P1429#2P1515#3P1579#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);

    // Pick up the left cup and drop it
    msg.data = "#0P1000#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(5);
    msg.data = "#0P1000#1P1257#2P1500#3P1735#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(5);
    msg.data = "#0P1000#1P1130#2P1292#3P1684#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1000#1P1130#2P1292#3P1684#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1000#1P1429#2P1515#3P1579#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1000#1P1429#2P1515#3P1579#4P1500#5P500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1000#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(5);

    // Push the first cup back to original position
    msg.data = "#0P1665#1P1280#2P1369#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1684#1P1299#2P1611#3P1808#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(5);
    msg.data = "#0P1500#1P1226#2P1503#3P1783#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(3);
    msg.data = "#0P1500#1P1500#2P1500#3P1500#4P1500#5P1500T2000\n";
    ROS_INFO("sending command : %s", msg.data.c_str());
    lRobotarmCommandPublisher.publish(msg);
    ros::spinOnce();
    sleep(5);
  }
  return 0;
}