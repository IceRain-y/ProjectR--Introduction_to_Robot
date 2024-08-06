#ifndef ROBOTARMCONTROLLER_H_
#define ROBOTARMCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>

#define MINSIMULATEDDEGREES -3.14
#define MAXSIMULATEDDEGREES 3.14
#define MINGRIPPERINVALUE 1
#define MAXGRIPPERINVALUE 4
#define MINGRIPPERDEGREES -0.02
#define MAXGRIPPERDEGREES 0.02
#define NUMBEROFGRIPPERPOSITIONS 5
#define MINPULSEWIDTH 500
#define MAXPULSEWIDTH 2500

class RobotarmController
{
  public:
    /**
     * @brief Construct a new Robotarm Controller object
     */
    RobotarmController();
    /**
     * @brief Destroy the Robotarm Controller object
     */
    ~RobotarmController();
    
    /**
     * @brief Updates the current mJointPositions based on the velocities
     */
    void updateRobotArmPosition();

  private:
    /**
      * @brief The nodehandler for the Subscriber and Publisher
      */
    ros::NodeHandle mNodeHandler;
    /**
     * @brief The subscriber that listens for robotarm commands
     */
    ros::Subscriber mRobotarmCommandSubscriber;
    /**
     * @brief The Publisher that publishes the robotarm position
     */
    ros::Publisher mRobotarmPublisher;
    /**
     * @brief Whether the robotarm has reached its goal position
     */
    bool mReachedPosition;

    /**
     * @brief Initialize the communication
     */
    void initializeCommunication();

    // Robotarm position variables
    std::vector<std::string> mJointNames;
    std::vector<double> mJointPositions;
    std::vector<double> mGoalPositions;
    std::vector<double> mJointVelocities;

    /**
     * @brief The time to take for the current move in milliseconds
     */
    unsigned int mMoveTime;
    /**
     * @brief The time passed in the current move in milliseconds
     */
    unsigned int mMoveCount;

    /**
     * @brief Timers for timing the move
     */
    std::chrono::high_resolution_clock::time_point mPreviousMoveTime;
    std::chrono::high_resolution_clock::time_point mMoveStartTime;

    /**
     * @brief Sends the current state of mJointStates to the visualizer
     */
    void sendCurrentStateToVisualizer();

    /**
     * @brief Handle the command from the client
     * @param aCommand - The command to parse and handle
     */
    void handleCommand(std::string aCommand);

    /**
     * @brief Callback function for the robotarmCommands, parses and handles the recieved command
     * @param aRobotarmCommand - A command for the robotarm
     */
    void robotarmCommandCallback(const std_msgs::String::ConstPtr& aRobotarmCommand);

      /**
     * @brief Maps the value from the input range to the output range
     * @param aDegree - The value to convert
     * @param aInMin - The minimum value of the input range
     * @param aInMax - The maximum value of the input range
     * @param aOutMin - The minimum value of the output range
     * @param aOutMax - The maximum value of the output range 
     * @return unsigned int - The converted value
     */
    public:
    double mapValues(double aDegree, double aInMin, double aInMax, double aOutMin, double aOutMax) const;
};

#endif