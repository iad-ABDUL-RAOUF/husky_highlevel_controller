#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:

	
	/*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void scanCallback(const sensor_msgs::LaserScan& message);

	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;
	//! ROS topic subscriber.
  ros::Subscriber scanSubscriber_;
  //! ROS topic name to subscribe to.
  std::string scanTopicName_;
	//! ROS subscribed topic queu size
	int scanTopicQueueSize_;
};

} /* namespace */
