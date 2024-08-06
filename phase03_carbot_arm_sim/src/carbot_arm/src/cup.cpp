#include "cup.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cup");

  if (argc >= 5)
  {
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
    Cup gCup("cup");
    gCup.handleCollision();
  }

  return 0;
}

Cup::Cup(std::string aTopic, float aOriginalX, float aOriginalY, float aOriginalZ) : mCupPosX(aOriginalX), mCupPosY(aOriginalY), mCupPosZ(aOriginalZ), mTopic(aTopic)
{
  mPastFrameTime = ros::Time::now();
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  publishCup();
}

Cup::~Cup()
{
}

void Cup::publishCup() const
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_link";
  static_transformStamped.child_frame_id = mTopic;
  static_transformStamped.transform.translation.x = mCupPosX;
  static_transformStamped.transform.translation.y = mCupPosY;
  static_transformStamped.transform.translation.z = mCupPosZ;
  tf2::Quaternion quat;

  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
}

void Cup::showCup(const COLORS::ColorState aColor) const
{
  visualization_msgs::Marker lMarker;
  lMarker.header.frame_id = mTopic;
  lMarker.header.stamp = ros::Time::now();
  lMarker.ns = "cupObject_" + mTopic;
  lMarker.id = 0;
  lMarker.type = visualization_msgs::Marker::CYLINDER;
  lMarker.action = visualization_msgs::Marker::ADD;

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
  lMarker.lifetime = ros::Duration();

  setColor(aColor, lMarker);

  marker_pub.publish(lMarker);
  ros::spinOnce();
}

void Cup::showMarker(const COLORS::ColorState aColor,const std::string& aFrameName,const float aOffset) const
{
  visualization_msgs::Marker lMarker;
  lMarker.header.frame_id = aFrameName;
  lMarker.header.stamp = ros::Time::now();
  lMarker.ns = "marker_" + aFrameName;
  lMarker.id = 0;
  lMarker.type = visualization_msgs::Marker::SPHERE;
  lMarker.action = visualization_msgs::Marker::ADD;

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
  lMarker.lifetime = ros::Duration();

  setColor(aColor, lMarker);

  marker_pub.publish(lMarker);
  ros::spinOnce();
}

void Cup::handleCollision()
{
  tf::Vector3 leftGripperOffset;
  tf::Vector3 rightGripperOffset;

  ros::Rate rate(300.0);
  while (n.ok())
  {
    //Mark the grippers
    showMarker(COLORS::RED, FRAME_NAME_GRIPPER_RIGHT, GRIPPER_DEPTH);
    showMarker(COLORS::RED, FRAME_NAME_GRIPPER_LEFT, GRIPPER_DEPTH * -1);
    
    tf::StampedTransform leftGripper;
    tf::StampedTransform rightGripper;
    tf::StampedTransform newPosLeft;
    tf::StampedTransform newPosRight;
    tf::StampedTransform cup;

    try
    {
      std::string topicFrame = std::string("/") + mTopic;
      listener.lookupTransform(WORLD_OBJECT, topicFrame, ros::Time(0), cup);
      listener.lookupTransform(topicFrame, FRAME_NAME_GRIPPER_LEFT, ros::Time(0), leftGripper);
      listener.lookupTransform(topicFrame, FRAME_NAME_GRIPPER_RIGHT, ros::Time(0), rightGripper);
      listener.lookupTransform(WORLD_OBJECT, FRAME_NAME_GRIPPER_LEFT, ros::Time(0), newPosLeft);
      listener.lookupTransform(WORLD_OBJECT, FRAME_NAME_GRIPPER_RIGHT, ros::Time(0), newPosRight);

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
      else
      {
        leftGripperOffset = leftGripper.getOrigin();
        rightGripperOffset = leftGripper.getOrigin();

        mColor = COLORS::RED;

        if (cup.getOrigin().z() > 0)
        {
          auto lTimePast = ros::Time::now() - mPastFrameTime;
          float lDropMultiplier = lTimePast.toSec() / calculateFallingTime(cup);
          mCupPosZ -= cup.getOrigin().z() * lDropMultiplier;
        }
      }
      publishCup();
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    showCup(mColor);

    mPastFrameTime = ros::Time::now();
    rate.sleep();
  }
}

void Cup::setColor(const COLORS::ColorState aColor, visualization_msgs::Marker& aMarker) const
{
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

bool Cup::isObjectInGripper(const tf::StampedTransform& aObject) const
{
  const float lWidthMargin = CUP_SIZE - GRIPPER_DEPTH;
  const float lHeightMargin = (CUP_SIZE * 2) - GRIPPER_DEPTH;

  return (aObject.getOrigin().y() > lWidthMargin * -1 && aObject.getOrigin().y() < lWidthMargin) && (aObject.getOrigin().x() > lWidthMargin * -1 && aObject.getOrigin().x() < lWidthMargin) && (aObject.getOrigin().z() > lHeightMargin * -1 && aObject.getOrigin().z() < lHeightMargin);
}

float Cup::calculateFallingTime(const tf::StampedTransform& aObject) const
{
  const float FORCE_OF_GRAVITY = 9.5;

  float lValue = 2 * aObject.getOrigin().z() / FORCE_OF_GRAVITY;
  return sqrt(lValue);
}