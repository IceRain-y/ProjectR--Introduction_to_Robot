#include "robotarmController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotarmController");
  RobotarmController lRobotarmController;

  while (ros::ok())
  {
    ros::spinOnce();
    lRobotarmController.updateRobotArmPosition();
  }
  return 0;
}

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

void RobotarmController::initializeCommunication()
{
  mRobotarmCommandSubscriber = mNodeHandler.subscribe("robotarmCommand", 1000, &RobotarmController::robotarmCommandCallback, this);
  mRobotarmPublisher = mNodeHandler.advertise<sensor_msgs::JointState>("joint_states", 1000);
}

void RobotarmController::sendCurrentStateToVisualizer()
{
  sensor_msgs::JointState lMessage;
  lMessage.header.frame_id = "/base_link";
  lMessage.header.stamp = ros::Time::now();
  lMessage.name = mJointNames;
  lMessage.position = mJointPositions;
  mRobotarmPublisher.publish(lMessage);
}

void RobotarmController::updateRobotArmPosition()
{
  // If the position has not been reached and the time has not elapsed
  auto lTime = std::chrono::high_resolution_clock::now();
  if (!mReachedPosition && mJointPositions != mGoalPositions && mMoveCount < mMoveTime)
  {
    // Set the current position + velocity
    auto lTime = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(lTime - mPreviousMoveTime).count() > 1)
    {
      for (int i = 0; i < mJointPositions.size(); i++)
      {
        mJointPositions.at(i) += mJointVelocities.at(i);
      }
      sendCurrentStateToVisualizer();
      mMoveCount++;
      mPreviousMoveTime = std::chrono::high_resolution_clock::now();
    }
  }
  else
  {
    mReachedPosition = true;
    sendCurrentStateToVisualizer();
  }
}

void RobotarmController::handleCommand(std::string aCommand)
{
  // Parse command, always parse for all the servo's
  std::string lOriginalString = aCommand;
  // Get the servo pulsewidths
  std::string lSubstring;
  mGoalPositions.clear();
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
  // Convert to angles
  for (int i = 0; i < (mGoalPositions.size() - 1); i++)
  {
    mGoalPositions.at(i) = mapValues(mGoalPositions.at(i), MINPULSEWIDTH, MAXPULSEWIDTH, MINSIMULATEDDEGREES, MAXSIMULATEDDEGREES);
  }
  // Convert to gripper
  mGoalPositions.at(5) = mGoalPositions.at(5) / (MAXPULSEWIDTH / NUMBEROFGRIPPERPOSITIONS);
  mGoalPositions.at(5) = mapValues(mGoalPositions.at(5), MINGRIPPERINVALUE, MAXGRIPPERINVALUE, MINGRIPPERDEGREES, MAXGRIPPERDEGREES);
  // Calculate differences per millisecond from current position
  for (int i = 0; i < mJointPositions.size(); i++)
  {
    mJointVelocities.at(i) = (mGoalPositions.at(i) - mJointPositions.at(i)) / mMoveTime;
  }
  mReachedPosition = false;
  mMoveCount = 0;
  mMoveStartTime = std::chrono::high_resolution_clock::now();
}

void RobotarmController::robotarmCommandCallback(const std_msgs::String::ConstPtr &aRobotarmCommand)
{
  ROS_INFO("Recieved robotarmCommand : %s", aRobotarmCommand->data.c_str());
  handleCommand(aRobotarmCommand->data);
}

double RobotarmController::mapValues(double aDegree, double aInMin, double aInMax, double aOutMin, double aOutMax) const
{
  return (aDegree - aInMin) * (aOutMax - aOutMin) / (aInMax - aInMin) + aOutMin;
}
