#ifndef CUP_HPP_
#define CUB_HPP_

#include <string>
#include <iostream>
#include <cstdio>
#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

// Cup size values
#define CUP_SIZE 0.04
#define GRIPPER_DEPTH 0.01

#define PUSH_TO_SIDE_COUNT 1
#define PICK_UP_COUNT 2
#define PICK_UP_OFFSET 0.005



//Frames
#define FRAME_NAME_GRIPPER_LEFT "/gripper_left"
#define FRAME_NAME_GRIPPER_RIGHT "/gripper_right"
#define WORLD_OBJECT "/base_link"

/**
 * @brief Namespace that contains the colors used
 */
namespace COLORS
{
  /**
   * @brief Enum for the used color values
   */
  enum ColorState
  {
    RED,
    GREEN
  };
};

/**
 * @brief Simulates a cup in the rviz simulation environment
 */
class Cup
{
public:
  /**
   * @brief Construct a new Cup object
   * 
   * @param aTopic - The name of the topic this object will publish itself to
   * @param aOriginalX - X location of the start position
   * @param aOriginalY - Y location of the start position
   * @param aOriginalZ - Z location of the start position
   */
  Cup(std::string aTopic, float aOriginalX = 0.4, float aOriginalY = 0.0, float aOriginalZ = 0.0);
  ~Cup();

  /**
   * @brief This function will detect collision and act appropriately
   */
  void handleCollision();

private:
  /**
   * @brief This function will create a frame where the cup object can be shown
   */
  void publishCup() const;

  /**
   * @brief Shows the cup in the simulated environment
   * @param aColor - The color of the cup
   */
  void showCup(const COLORS::ColorState aColor) const;

  /**
   * @brief This function will draw a sphere
   * 
   * @param aColor - The color you want your sphere to be
   * @param aFrameName - The frame name you would like the sphere to be drawn in
   */
  void showMarker(const COLORS::ColorState aColor, const std::string& aFrameName, const float aOffset) const;

  /**
   * @brief Set the Color of the cup
   * 
   * @param aColor - The color you would like your object to be
   * @param aMarker - A reference of the object you want to color
   */
  void setColor(const COLORS::ColorState aColor, visualization_msgs::Marker& aMarker) const;

  /**
   * @brief This function will check if the object is touching the gripper
   * 
   * @param aObject - The distance from the cup relative from the gripper
   * @return true - Gripper is touching the object
   * @return false - Gripper is not touching the object
   */
  bool isObjectInGripper(const tf::StampedTransform& aObject) const;

  /**
   * @brief This function will calculate the falling speed of the object
   * 
   * @param object - The location of the object relative to the world
   * @return float - The falling time of the object in seconds before it hits the ground
   */
  float calculateFallingTime(const tf::StampedTransform& aObject) const;

  /**
   * @brief Topic name of the object
   */
  std::string mTopic;

  /**
   * @brief The x,y and z positions of the cup
   */
  float mCupPosX;
  float mCupPosY;
  float mCupPosZ;

  /**
   * @brief The set color your cup wil be displayed
   */
  COLORS::ColorState mColor = COLORS::RED;

  /**
   * @brief The time when the last frame was displayed. This valiable is used to know how much time is passed between the current frame and the past frame
   */
  ros::Time mPastFrameTime;

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  tf::TransformListener listener;
};

#endif