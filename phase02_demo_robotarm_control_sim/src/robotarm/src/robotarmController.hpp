//头文件保护
#ifndef ROBOTARMCONTROLLER_H_
#define ROBOTARMCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <cstring>
#include <string>
#include <vector>
// C++11 引入的，用于提供日期和时间的处理功能，特别是高精度的时钟和持续时间
#include <chrono>

//定义了模拟关节角度的最小值和最大值，这里使用的是弧度制，范围从-π到π。
#define MINSIMULATEDDEGREES -3.14
#define MAXSIMULATEDDEGREES 3.14
//定义了夹爪（gripper）的输入值的最小和最大范围，这个范围可能用于控制夹爪的张开和闭合程度
#define MINGRIPPERINVALUE 1
#define MAXGRIPPERINVALUE 4
//定义了夹爪角度的最小和最大变化范围，以弧度为单位。这些值可能用于精确控制夹爪的微小移动
#define MINGRIPPERDEGREES -0.02
#define MAXGRIPPERDEGREES 0.02
//定义了夹爪可以停留的不同位置的数量。这个值可能用于在编程中枚举或索引夹爪的不同状态
#define NUMBEROFGRIPPERPOSITIONS 5
//定义了脉冲宽度的最小和最大值，这可能用于控制电机或其他执行器的驱动信号。
#define MINPULSEWIDTH 500
#define MAXPULSEWIDTH 2500

class RobotarmController
{
  public:
    /**
     * @brief Construct a new Robotarm Controller object
     */
    //构造函数
    RobotarmController();
    /**
     * @brief Destroy the Robotarm Controller object
     */
    //析构函数
    ~RobotarmController();
    
    /**
     * @brief Updates the current mJointPositions based on the velocities
     */
    //根据当前的关节速度和时间更新机械臂的关节位置
    void updateRobotArmPosition();

  private:
    /**
      * @brief The nodehandler for the Subscriber and Publisher
      */
    //创建节点句柄
    ros::NodeHandle mNodeHandler;
    /**
     * @brief The subscriber that listens for robotarm commands
     */
    //创建订阅者消息
    ros::Subscriber mRobotarmCommandSubscriber;
    /**
     * @brief The Publisher that publishes the robotarm position
     */
    //创建发布者消息
    ros::Publisher mRobotarmPublisher;
    /**
     * @brief Whether the robotarm has reached its goal position
     */
    //用于指示机器人手臂是否已达到其目标位置。
    bool mReachedPosition;

    /**
     * @brief Initialize the communication
     */
    //负责设置ROS的订阅者和发布者，包括设置主题名称、消息队列大小和回调函数。
    //您可能还需要在这里订阅/joint_states主题（或其他类似主题），以便获取机械臂的当前状态
    void initializeCommunication();

    // Robotarm position variables
    //用于存储机器人手臂上每个关节的名称
    std::vector<std::string> mJointNames;
    //用于存储机器人手臂上每个关节的当前位置（如角度或位置）
    std::vector<double> mJointPositions;
    //用于存储机器人手臂上每个关节的目标位置
    std::vector<double> mGoalPositions;
    //用于存储机器人手臂上每个关节的当前速度（如角速度或线速度）
    std::vector<double> mJointVelocities;

    /**
     * @brief The time to take for the current move in milliseconds
     */
    //用于指定当前移动操作所需的总时间
    //这个时间可以是从命令接收时开始到机器人手臂应该到达目标位置所需的预估时间。
    unsigned int mMoveTime;
    /**
     * @brief The time passed in the current move in milliseconds
     */
    //用于跟踪当前移动操作已经过去的时间（以毫秒为单位）。
    //这个变量在移动开始时被初始化为0，并随着时间的推移而递增，直到达到mMoveTime所指定的总时间。
    unsigned int mMoveCount;

    /**
     * @brief Timers for timing the move
     */
    //这个变量用于记录上一次移动操作结束的时间点。它可以在移动操作完成后被更新，以便在需要时能够回溯上一次移动的时间信息。
    std::chrono::high_resolution_clock::time_point mPreviousMoveTime;
    //这个变量用于记录当前移动操作开始的时间点。它应该在移动操作开始时被初始化，以便能够计算移动操作的持续时间。
    std::chrono::high_resolution_clock::time_point mMoveStartTime;

    /**
     * @brief Sends the current state of mJointStates to the visualizer
     */
    //负责将机械臂的当前状态（如关节位置、速度等）发送到可视化工具。
    //您可能需要使用mRobotarmPublisher来发布一个包含这些信息的消息
    void sendCurrentStateToVisualizer();

    /**
     * @brief Handle the command from the client
     * @param aCommand - The command to parse and handle
     */
    //解析传入的命令字符串，并根据命令的内容执行相应的操作。
    void handleCommand(std::string aCommand);

    /**
     * @brief Callback function for the robotarmCommands, parses and handles the recieved command
     * @param aRobotarmCommand - A command for the robotarm
     */
    //用于处理接收到的机器人手臂命令消息
    void robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand);

      /**
     * @brief Maps the value from the input range to the output range
     * @param aDegree - The value to convert-需要转换的值
     * @param aInMin - The minimum value of the input range-输入范围的最小值
     * @param aInMax - The maximum value of the input range-输入范围的最大值
     * @param aOutMin - The minimum value of the output range-输出范围的最小值
     * @param aOutMax - The maximum value of the output range-输出范围的最大值
     * @return unsigned int - The converted value-转换后的值
     */
    public:
    //用于将输入范围内的值映射到输出范围内的值。
    double mapValues(double aDegree, double aInMin, double aInMax, double aOutMin, double aOutMax) const;
};

#endif