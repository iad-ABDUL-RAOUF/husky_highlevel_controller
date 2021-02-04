#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <cmath>

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
  return true;
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& message)
{
  float minVal = std::numeric_limits<float>::infinity();
  for (unsigned int i = 0; i< message.ranges.size(); i++)
  {
    if (std::isnormal(message.ranges[i]) && (message.range_min < message.ranges[i]) && (message.ranges[i] < message.range_max))
    {
      minVal = std::min(minVal, message.ranges[i]);
    }
  }
  ROS_INFO_STREAM("min measured range is : " << minVal << std::endl);
}

} /* namespace */
