#include "cup.hpp"

//主函数
int main(int argc, char **argv)
{
  //初始化ROS节点
  ros::init(argc, argv, "cup");

  //处理命令行参数
  if (argc >= 5)
  {
    //表示位置和名字
    float lArgX, lArgY, lArgZ;
    lArgX = atof(argv[1]);
    lArgY = atof(argv[2]);
    lArgZ = atof(argv[3]);
    std::string lArgName = std::string(argv[4]);

    Cup gCup(lArgName, lArgX, lArgY, lArgZ);
    gCup.handleCollision();
  }
  else
  {
    //默认情况设置：命令行参数不足时，创建一个gcup对象，调用handleCollision
    Cup gCup("cup");
    gCup.handleCollision();
  }

  return 0;
}

//构造函数
//使用了构造函数初始化列表来初始化类的成员变量
Cup::Cup(std::string aTopic, float aOriginalX, float aOriginalY, float aOriginalZ) : mCupPosX(aOriginalX), mCupPosY(aOriginalY), mCupPosZ(aOriginalZ), mTopic(aTopic)
{
  //初始化ROS时间戳为当前时间
  mPastFrameTime = ros::Time::now();
  // 创建一个发布者，用于发布可视化标记（如Rviz中的标记）  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // 调用成员函数来发布当前杯子的信息
  publishCup();
}

Cup::~Cup()
{
}

void Cup::publishCup() const
{
  //创建一个静态的tf2_ros::StaticTransformBroadcaster对象，该对象在函数第一次被调用时初始化，  
  // 并在程序的生命周期内保持不变。由于它是静态的，所以它在所有Cup对象的实例之间共享。  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  //创建一个geometry_msgs::TransformStamped消息，用于存储和广播变换信息。
  geometry_msgs::TransformStamped static_transformStamped;
  //设置消息的时间戳为当前时间。
  static_transformStamped.header.stamp = ros::Time::now();
  // 设置消息的父框架ID为"base_link"，这通常是机器人基座的框架。
  static_transformStamped.header.frame_id = "base_link";
  // 设置消息的子框架ID为Cup对象的mTopic成员变量。这里假设mTopic包含了一个有效的框架名称。  
  // 注意：这可能导致问题，因为通常mTopic应该是用于通信的主题名，而不是框架名。  
  static_transformStamped.child_frame_id = mTopic;
  // 设置变换的平移部分，即杯子在父框架中的位置。
  static_transformStamped.transform.translation.x = mCupPosX;
  static_transformStamped.transform.translation.y = mCupPosY;
  static_transformStamped.transform.translation.z = mCupPosZ;
  // 创建一个tf2::Quaternion对象来表示旋转，并将其设置为零旋转（即没有旋转）
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  // 将四元数转换为geometry_msgs::Quaternion类型，并设置到变换的旋转部分。
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  // 使用静态变换广播器发送变换消息。  
  // 注意：由于使用了静态广播器，这个变换只会在第一次调用publishCup时发送，  
  // 除非ROS节点被重启或静态广播器被显式重置。 
  static_broadcaster.sendTransform(static_transformStamped);
}

void Cup::showCup(const COLORS::ColorState aColor) const
{
  //构建和发送可视化标记
  visualization_msgs::Marker lMarker;
  //设置Header：
  //设置Marker的参考坐标系ID
  lMarker.header.frame_id = mTopic;
  //设置时间戳为当前时间
  lMarker.header.stamp = ros::Time::now();
  //设置命名空间(ns)和ID
  //设置Marker的命名空间，这里通过mTopic来区分不同的杯子对象
  lMarker.ns = "cupObject_" + mTopic;
  //设置Marker的ID
  lMarker.id = 0;
  //指定Marker的形状为圆柱体，适用于表示杯子
  lMarker.type = visualization_msgs::Marker::CYLINDER;
  //指定动作为添加，即向ROS的可视化环境中添加这个Marker
  lMarker.action = visualization_msgs::Marker::ADD;

  //设置位置，姿态，尺寸
  lMarker.pose.position.x = 0;
  lMarker.pose.position.y = 0;
  lMarker.pose.position.z = CUP_SIZE;
  lMarker.pose.orientation.x = 0;
  lMarker.pose.orientation.y = 0;
  lMarker.pose.orientation.z = 0;
  lMarker.pose.orientation.w = 1.0;
  lMarker.scale.x = CUP_SIZE;
  lMarker.scale.y = CUP_SIZE;
  lMarker.scale.z = CUP_SIZE * 2;
  //设置生命周期
  lMarker.lifetime = ros::Duration();
  //设置颜色
  setColor(aColor, lMarker);
  //发布Marker
  marker_pub.publish(lMarker);
  //循环一次
  ros::spinOnce();
}

//在ROS环境中发布一个表示球体的消息
void Cup::showMarker(const COLORS::ColorState aColor,const std::string& aFrameName,const float aOffset) const
{
  //初始化Marker消息：用于存储和发送标记信息
  visualization_msgs::Marker lMarker;
  //设置Header
  //将标记的参考坐标系ID设置为传入的aFrameName
  lMarker.header.frame_id = aFrameName;
  //设置时间戳为当前时间
  lMarker.header.stamp = ros::Time::now();
  //设置命名空间(ns)和ID
  //为标记设置一个命名空间，以便在ROS的可视化界面中区分不同的标记
  lMarker.ns = "marker_" + aFrameName;
  lMarker.id = 0;
  //设置Marker的类型和动作
  //指定标记的形状为球体
  lMarker.type = visualization_msgs::Marker::SPHERE;
  //指定动作为添加
  lMarker.action = visualization_msgs::Marker::ADD;

  //设置位置，姿态，尺寸
  lMarker.pose.position.x = 0;
  lMarker.pose.position.y = aOffset;
  lMarker.pose.position.z = GRIPPER_DEPTH;
  lMarker.pose.orientation.x = 0;
  lMarker.pose.orientation.y = 0;
  lMarker.pose.orientation.z = 0;
  lMarker.pose.orientation.w = 1.0;
  lMarker.scale.x = 0.005;
  lMarker.scale.y = 0.005;
  lMarker.scale.z = 0.005;
  //设置生命周期
  lMarker.lifetime = ros::Duration();
  //设置颜色
  setColor(aColor, lMarker);
  //发布消息
  marker_pub.publish(lMarker);
  //循环一次
  ros::spinOnce();
}

//处理杯子（或其他物体）与机械臂抓手（左右两个）之间的碰撞检测，并根据检测结果来更新杯子的位置信息以及颜色状态
void Cup::handleCollision()
{
  //初始化变量：用来存储左右抓手相对于某个参考点（如机械臂基座或世界坐标系）的偏移量
  tf::Vector3 leftGripperOffset;
  tf::Vector3 rightGripperOffset;
  //设置循环的速率为300Hz，即每秒循环300次
  ros::Rate rate(300.0);

  //循环检测
  //使用while (n.ok())循环来持续检测和处理碰撞
  while (n.ok())
  {
    //Mark the grippers
    //使用showMarker函数在ROS的可视化界面中标记左右抓手的位置。
    showMarker(COLORS::RED, FRAME_NAME_GRIPPER_RIGHT, GRIPPER_DEPTH);
    showMarker(COLORS::RED, FRAME_NAME_GRIPPER_LEFT, GRIPPER_DEPTH * -1);
    
    //存储坐标变换关系
    //左右两个机械臂抓手坐标系的变换
    tf::StampedTransform leftGripper;
    tf::StampedTransform rightGripper;
    //表示左右抓手的新位置
    tf::StampedTransform newPosLeft;
    tf::StampedTransform newPosRight;
    //用于存储从世界坐标系或其他参考坐标系到杯子当前位置的变换
    tf::StampedTransform cup;

    try
    {
    //获取变换信息
    //使用listener.lookupTransform来获取不同坐标系之间的变换关系
    //从世界坐标系到物体坐标系、从物体坐标系到左右抓手坐标系、以及从世界坐标系到左右抓手坐标系的变换。
      std::string topicFrame = std::string("/") + mTopic;
      listener.lookupTransform(WORLD_OBJECT, topicFrame, ros::Time(0), cup);
      listener.lookupTransform(topicFrame, FRAME_NAME_GRIPPER_LEFT, ros::Time(0), leftGripper);
      listener.lookupTransform(topicFrame, FRAME_NAME_GRIPPER_RIGHT, ros::Time(0), rightGripper);
      listener.lookupTransform(WORLD_OBJECT, FRAME_NAME_GRIPPER_LEFT, ros::Time(0), newPosLeft);
      listener.lookupTransform(WORLD_OBJECT, FRAME_NAME_GRIPPER_RIGHT, ros::Time(0), newPosRight);

    //碰撞检测与位置更新
      //计算杯子位置：当物体被两个抓手同时抓住时，计算杯子在左右抓手中心点的平均位置。
      if (isObjectInGripper(leftGripper) + isObjectInGripper(rightGripper) == PICK_UP_COUNT)
      {
        mCupPosX = (newPosLeft.getOrigin().x() + newPosRight.getOrigin().x()) / 2;
        mCupPosY = (newPosLeft.getOrigin().y() + newPosRight.getOrigin().y()) / 2;

        auto lNewCupPosZ = ((newPosLeft.getOrigin().z() + newPosRight.getOrigin().z()) / 2) - rightGripperOffset.z() + PICK_UP_OFFSET;

        if (lNewCupPosZ > 0)
          mCupPosZ = lNewCupPosZ;

        mColor = COLORS::GREEN;
      }else if(isObjectInGripper(leftGripper) + isObjectInGripper(rightGripper) == PUSH_TO_SIDE_COUNT)
      {
        //当只有一个抓手抓住物体时，更新了杯子的位置以反映该抓手的位置
        if(isObjectInGripper(leftGripper))
        {
        mCupPosX = (newPosLeft.getOrigin().x() - leftGripperOffset.x());
        mCupPosY = (newPosLeft.getOrigin().y() - leftGripperOffset.y());
        }else
        {
        mCupPosX = (newPosRight.getOrigin().x() + rightGripperOffset.x());
        mCupPosY = (newPosRight.getOrigin().y() + rightGripperOffset.y());
        }
        
      }
      //处理未抓住的情况
      else
      {
        leftGripperOffset = leftGripper.getOrigin();
        rightGripperOffset = leftGripper.getOrigin();

        mColor = COLORS::RED;

        //实现了一个简单的物理模拟，当杯子没有被抓住且其Z坐标大于0时，根据时间差和计算出的下落时间来更新杯子的Z坐标
        if (cup.getOrigin().z() > 0)
        {
          auto lTimePast = ros::Time::now() - mPastFrameTime;
          float lDropMultiplier = lTimePast.toSec() / calculateFallingTime(cup);
          mCupPosZ -= cup.getOrigin().z() * lDropMultiplier;
        }
      }
      publishCup();
    }
    //错误处理和循环控制：
    //使用try-catch块来捕获tf::TransformException，这是处理tf变换查找中可能出现的错误的好方法
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    showCup(mColor);

    mPastFrameTime = ros::Time::now();
    //在每次循环迭代结束时，您调用rate.sleep确保()循环来以指定的速率运行。
    rate.sleep();
  }
}

//设置Marker对象的颜色
void Cup::setColor(const COLORS::ColorState aColor, visualization_msgs::Marker& aMarker) const
{
  //修改了传入的aMarker对象的属性：红或绿
  switch (aColor)
  {
  case COLORS::ColorState::RED:
    aMarker.color.r = 1.0f;
    aMarker.color.g = 0.0f;
    aMarker.color.b = 0.0f;
    aMarker.color.a = 1.0;
    break;

  case COLORS::ColorState::GREEN:
    aMarker.color.r = 0.0f;
    aMarker.color.g = 1.0f;
    aMarker.color.b = 0.0f;
    aMarker.color.a = 1.0;
    break;
  }
}

//判断目标是否位于机械手的夹持器内
bool Cup::isObjectInGripper(const tf::StampedTransform& aObject) const
{
  //通过 杯子的尺寸 减去 夹持器的深度 来计算的。这个值代表了夹持器在 X 和 Y 方向上的有效宽度的一半（因为考虑了正负方向）。
  const float lWidthMargin = CUP_SIZE - GRIPPER_DEPTH;

  //这里假设 CUP_SIZE 的两倍代表了夹持器在 Z 轴方向上的总高度（或有效抓取高度），然后从中减去夹持器的深度来得到有效抓取高度的一半（同样考虑了正负方向）
  const float lHeightMargin = (CUP_SIZE * 2) - GRIPPER_DEPTH;

  //通过比较 aObject 的原点坐标（getOrigin().x(), getOrigin().y(), getOrigin().z()）与这些边距来判断物体是否在夹持器内。
  //具体来说，它检查物体的 X、Y、Z 坐标是否都位于各自的有效抓取区域内
  return (aObject.getOrigin().y() > lWidthMargin * -1 && aObject.getOrigin().y() < lWidthMargin) && (aObject.getOrigin().x() > lWidthMargin * -1 && aObject.getOrigin().x() < lWidthMargin) && (aObject.getOrigin().z() > lHeightMargin * -1 && aObject.getOrigin().z() < lHeightMargin);
}

//计算目标从当前位置自由落体到地面的时间
float Cup::calculateFallingTime(const tf::StampedTransform& aObject) const
{
  //设置重力加速度
  const float FORCE_OF_GRAVITY = 9.5;

  //自由落体公式：d = 0.5 * g * (t ^ 2)
  float lValue = 2 * aObject.getOrigin().z() / FORCE_OF_GRAVITY;
  return sqrt(lValue);
}