#include "robotarmController.hpp"

int main(int argc, char **argv)
{
  //初始化节点句柄
  ros::init(argc, argv, "robotarmController");
  //创建控制器对象
  RobotarmController lRobotarmController;
  //ROS系统还在运行时，调用控制器对象来更新机器人手臂的位置
  while (ros::ok())
  {
    ros::spinOnce();
    lRobotarmController.updateRobotArmPosition();
  }
  return 0;
}

//构造函数，初始化机器人手臂控制器的一些关键成员变量，并调用initializeCommunication函数来初始化与机器人手臂的通信
RobotarmController::RobotarmController() : mJointNames{"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand"},
                                           mJointPositions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                           mJointVelocities{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                           mGoalPositions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                           mReachedPosition(false),
                                           mMoveTime(0),
                                           mMoveCount(0),
                                           mMoveStartTime(std::chrono::high_resolution_clock::now()),
                                           mPreviousMoveTime(std::chrono::high_resolution_clock::now())
{
  initializeCommunication();
}

RobotarmController::~RobotarmController()
{
}


//初始化与机器人手臂通信的ROS话题订阅者和发布者
void RobotarmController::initializeCommunication()
{
  //订阅机器人手臂命令话题
  mRobotarmCommandSubscriber = mNodeHandler.subscribe("robotarmCommand", 1000, &RobotarmController::robotarmCommandCallback, this);
  //发布机器人手臂的关节状态
  mRobotarmPublisher = mNodeHandler.advertise<sensor_msgs::JointState>("joint_states", 1000);
}

//负责将机器人手臂的当前关节状态发送到一个可视化器
void RobotarmController::sendCurrentStateToVisualizer()
{
  //创建sensor_msgs::JointState消息。专门用于表示一组关节的状态，包括它们的名称、位置、速度等
  sensor_msgs::JointState lMessage;
  //设置消息头：消息的参考坐标系ID，消息的时间戳为当前时间
  lMessage.header.frame_id = "/base_link";
  lMessage.header.stamp = ros::Time::now();
  //填充关节名称和位置：
  lMessage.name = mJointNames;
  lMessage.position = mJointPositions;
  //发布消息
  mRobotarmPublisher.publish(lMessage);
}

//负责更新机器人手臂的位置，直到它达到目标位置或达到最大移动时间
void RobotarmController::updateRobotArmPosition()
{
  // If the position has not been reached and the time has not elapsed
  //获取当前时间
  auto lTime = std::chrono::high_resolution_clock::now();
  //检查是否达到目标位置或最大移动时间：判断是否尚未到达目标位置，判断当前位置不是目标位置，判断尚未到达最大移动次数和时间限制
  if (!mReachedPosition && mJointPositions != mGoalPositions && mMoveCount < mMoveTime)
  {
    // Set the current position + velocity
    //获取高精度时间
    auto lTime = std::chrono::high_resolution_clock::now();
    //如果当前时间距离上一次移动时间超过了1毫秒
    if (std::chrono::duration_cast<std::chrono::milliseconds>(lTime - mPreviousMoveTime).count() > 1)
    {
      //遍历所有关节，将每个关节的当前位置增加对应的速度
      for (int i = 0; i < mJointPositions.size(); i++)
      {
        mJointPositions.at(i) += mJointVelocities.at(i);
      }
      //调用函数将更新后的位置发送到可视化器
      sendCurrentStateToVisualizer();
      //增加移动计数
      mMoveCount++;
      //更新当前时间，以便在下一次迭代中检查是否已经过了足够的时间来再次更新位置
      mPreviousMoveTime = std::chrono::high_resolution_clock::now();
    }
  }
  else
  {
    //处理已达到目标位置或最大移动时间
    mReachedPosition = true;
    //调用函数发送最终的位置信息到可视化器
    sendCurrentStateToVisualizer();
  }
}

//负责处理一个表示机器人手臂目标的命令字符串，并据此更新目标位置、移动时间以及（可能）夹爪的位置。
void RobotarmController::handleCommand(std::string aCommand)
{
  //解析命令字符串
  // Parse command, always parse for all the servo's
  std::string lOriginalString = aCommand;
  // Get the servo pulsewidths
  std::string lSubstring;
  mGoalPositions.clear();
  //通过查找#字符来分割字符串
  //使用atoi函数将找到的脉冲宽度字符串转换为整数，然后存储到mGoalPositions向量
  lSubstring = lOriginalString.substr(lOriginalString.find("#") + 1);
  mGoalPositions.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  mGoalPositions.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  mGoalPositions.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  mGoalPositions.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  mGoalPositions.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("#")).c_str()));
  lSubstring = lSubstring.substr(lSubstring.find("#") + 1);
  mGoalPositions.push_back(atoi(lSubstring.substr(lSubstring.find("P") + 1, lSubstring.find("T")).c_str()));
  // Get the time
  mMoveTime = atoi(lSubstring.substr(lSubstring.find("T") + 1).c_str());
  //将脉冲宽度转换为角度：脉冲宽度信号控制伺服电机转动，代码需要将脉冲宽度转换为对应的模拟角度
  // Convert to angles
  for (int i = 0; i < (mGoalPositions.size() - 1); i++)
  {
    //通过mapValues函数实现，该函数接受当前值、当前值的范围（脉冲宽度范围）以及目标值的范围（模拟角度范围），并返回转换后的值。
    mGoalPositions.at(i) = mapValues(mGoalPositions.at(i), MINPULSEWIDTH, MAXPULSEWIDTH, MINSIMULATEDDEGREES, MAXSIMULATEDDEGREES);
  }
  //夹爪索引
  // Convert to gripper
  mGoalPositions.at(5) = mGoalPositions.at(5) / (MAXPULSEWIDTH / NUMBEROFGRIPPERPOSITIONS);
  mGoalPositions.at(5) = mapValues(mGoalPositions.at(5), MINGRIPPERINVALUE, MAXGRIPPERINVALUE, MINGRIPPERDEGREES, MAXGRIPPERDEGREES);
  // Calculate differences per millisecond from current position
  //计算关节速度
  for (int i = 0; i < mJointPositions.size(); i++)
  {
    mJointVelocities.at(i) = (mGoalPositions.at(i) - mJointPositions.at(i)) / mMoveTime;
  }
  mReachedPosition = false;
  mMoveCount = 0;
  mMoveStartTime = std::chrono::high_resolution_clock::now();
}

//用于处理机器人手臂的命令
void RobotarmController::robotarmCommandCallback(const std_msgs::String::ConstPtr &aRobotarmCommand)
{
  //接收并打印命令：
  ROS_INFO("Recieved robotarmCommand : %s", aRobotarmCommand->data.c_str());
  //处理命令
  handleCommand(aRobotarmCommand->data);
}

//将一个输入值（在这个例子中是一个角度值aDegree，它在aInMin和aInMax之间）映射（或转换）到一个新的输出范围内（aOutMin到aOutMax）
double RobotarmController::mapValues(double aDegree, double aInMin, double aInMax, double aOutMin, double aOutMax) const
{
  //线性插值
  return (aDegree - aInMin) * (aOutMax - aOutMin) / (aInMax - aInMin) + aOutMin;
}
