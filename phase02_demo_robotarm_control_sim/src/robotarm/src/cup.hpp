//头文件保护，仅包含一次
#ifndef CUP_HPP_
#define CUP_HPP_
//标准头文件和ROS文件
#include <string>
#include <iostream>
#include <cstdio>
#include <ros/ros.h>

//ROS变换和消息类型的头文件引用
//tf2提供了四元数的表示和运算
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
//监听变换
#include <tf/transform_listener.h>
//定义了与几何和速度相关的消息类型，
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
//在RVIZ中显示标记（如箭头、立方体等）
#include <visualization_msgs/Marker.h>

// Cup size values
//杯子尺寸，末端执行器深度，推向一侧的计数，拾取次数，拾取偏移量
#define CUP_SIZE 0.04
#define GRIPPER_DEPTH 0.01
#define PUSH_TO_SIDE_COUNT 1
#define PICK_UP_COUNT 2
#define PICK_UP_OFFSET 0.005


//Frames
//机械臂左手爪，机械臂右手爪，参考坐标系
#define FRAME_NAME_GRIPPER_LEFT "/gripper_left"
#define FRAME_NAME_GRIPPER_RIGHT "/gripper_right"
#define WORLD_OBJECT "/base_link"

/**
 * @brief Namespace that contains the colors used
 */
//颜色的命名空间，放在被抓取的物体上用作标记
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

  //处理碰撞相关的函数
  void handleCollision();

private:
  /**
   * @brief This function will create a frame where the cup object can be shown
   */
  //用于在ROS中发布杯子的位置或状态信息
  void publishCup() const;

  /**
   * @brief Shows the cup in the simulated environment
   * @param aColor - The color of the cup
   */
  //模拟环境中显示杯子，并可以根据传入的参数aColor来改变杯子的颜色
  void showCup(const COLORS::ColorState aColor) const;

  /**
   * @brief This function will draw a sphere
   * 
   * @param aColor - The color you want your sphere to be
   * @param aFrameName - The frame name you would like the sphere to be drawn in
   */
  //用于在指定坐标系中绘制一个球体标记，并可以设置其颜色和偏移量（aOffset）。
  //这个函数可能用于显示不同类型的对象，而不仅仅是杯子。
  void showMarker(const COLORS::ColorState aColor, const std::string& aFrameName, const float aOffset) const;

  /**
   * @brief Set the Color of the cup
   * 
   * @param aColor - The color you would like your object to be
   * @param aMarker - A reference of the object you want to color
   */
  //用于设置ROS可视化标记的颜色。它接受一个颜色状态（aColor）和一个标记的引用（aMarker），并修改该标记的颜色属性
  void setColor(const COLORS::ColorState aColor, visualization_msgs::Marker& aMarker) const;

  /**
   * @brief This function will check if the object is touching the gripper
   * 
   * @param aObject - The distance from the cup relative from the gripper
   * @return true - Gripper is touching the object
   * @return false - Gripper is not touching the object
   */
  //检查一个对象（通过其相对于机器人的变换aObject表示）是否位于机器人的末端执行器（如机械手爪）中。
  //它返回一个布尔值，指示对象是否被抓取。
  bool isObjectInGripper(const tf::StampedTransform& aObject) const;

  /**
   * @brief This function will calculate the falling speed of the object
   * 
   * @param object - The location of the object relative to the world
   * @return float - The falling time of the object in seconds before it hits the ground
   */
  //计算一个对象（aObject）从当前位置自由落体到地面所需的时间。
  //它可能基于对象的位置和重力加速度来计算这个时间
  float calculateFallingTime(const tf::StampedTransform& aObject) const;

  /**
   * @brief Topic name of the object
   */
  //存储了与杯子相关的ROS话题的名称。这可能用于订阅或发布与杯子位置、状态或颜色相关的信息。
  std::string mTopic;

  /**
   * @brief The x,y and z positions of the cup
   */
  //分别存储了杯子在三维空间中的X、Y、Z坐标位置
  float mCupPosX;
  float mCupPosY;
  float mCupPosZ;

  /**
   * @brief The set color your cup wil be displayed
   */
  //枚举变量表示杯子的颜色，并默认设置为红色。COLORS::ColorState可能是一个自定义的枚举类型，用于表示不同的颜色状态
  COLORS::ColorState mColor = COLORS::RED;

  /**
   * @brief The time when the last frame was displayed. This valiable is used to know how much time is passed between the current frame and the past frame
   */
  //共同构成了ROS节点中与时间跟踪、节点管理、消息发布和坐标变换监听相关的基本组件。
  //在ROS应用程序中，这些组件通常会被用来实现复杂的机器人行为，如导航、物体识别、状态监控等
  //用于存储过去某个时间点的时间戳
  ros::Time mPastFrameTime;
  //是ROS中非常重要的一个类，它是ROS节点与ROS系统其他部分进行交互的接口。
  ros::NodeHandle n;
  //用于发布消息到ROS话题
  ros::Publisher marker_pub;
  //tf库中的一个类，用于监听坐标变换信息
  tf::TransformListener listener;
};

#endif