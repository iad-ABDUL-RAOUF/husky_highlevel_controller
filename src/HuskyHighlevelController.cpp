#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


namespace husky_highlevel_controller
{

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  scanSubscriber_ = nodeHandle_.subscribe(scanTopicName_, scanTopicQueueSize_, &HuskyHighlevelController::scanCallback, this);
  commandPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, this);
  visPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 10, this);

  ROS_INFO("Successfully launched node husky_highlevel_controller.");
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

bool HuskyHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("scan_topic_name", scanTopicName_))
  {
    return false;
  }
  if (!nodeHandle_.getParam("scan_topic_queue_size", scanTopicQueueSize_))
  {
    return false;
  }
  if (!nodeHandle_.getParam("controller_proportional_gain", Kp_))
  {
    return false;
  }
  return true;
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& message)
{
  float minVal = std::numeric_limits<float>::infinity();
  float pillarDirection = 0;
  float dStopControlleur = 1;
  float pillarDistance = 0;
  unsigned int count_point = 0;
  for (unsigned int i = 0; i< message.ranges.size(); i++)
  {
    if (std::isnormal(message.ranges[i]) && (message.range_min < message.ranges[i]) && (message.ranges[i] < message.range_max))
    {
      minVal = std::min(minVal, message.ranges[i]);

      pillarDirection = pillarDirection + message.angle_min + i*message.angle_increment;
      pillarDistance += message.ranges[i];
      count_point += 1;
      //ROS_INFO_STREAM("point direction: " << message.angle_min + i*message.angle_increment << rad);

    }
  }
  if (count_point == 0)
  {
    ROS_INFO("no valid points in scan");
    pillarDirection = M_PI;
    pillarDistance = dStopControlleur+1;
  }
  else
  {
    pillarDirection = pillarDirection/count_point;
    pillarDistance = pillarDistance/count_point;
  }
  ROS_INFO_STREAM("count_point : " << count_point);
  ROS_INFO_STREAM("min measured range is : " << minVal << " m");
  ROS_INFO_STREAM("pillar distance is : " << pillarDistance << " m");
  ROS_INFO_STREAM("pillar direction is : " << pillarDirection << " rad");

  //simple controleur proportionnel
  geometry_msgs::Twist commande;
  if (pillarDistance > dStopControlleur)
  {
    commande.linear.x = Kp_*(pillarDistance-dStopControlleur);
    commande.linear.y = 0;
    commande.linear.z = 0;
    commande.angular.x = 0;
    commande.angular.y = 0;
    commande.angular.z = -Kp_*pillarDirection;
    commandPublisher_.publish(commande);
  }
  
  // creation du marker a envoyer dans Rviz pour l'affichage du pillier detecté
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_laser";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pillarDistance;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  visPub_.publish(marker);
}

} /* namespace */
