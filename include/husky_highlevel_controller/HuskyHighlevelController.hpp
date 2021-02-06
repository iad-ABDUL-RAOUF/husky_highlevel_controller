#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

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
	//! ROS topic publisher
	ros::Publisher commandPublisher_;
	ros::Publisher visPub_;
	//! gain proportionnel du controlleur
	float Kp_;
  //! ROS topic name to subscribe to.
  std::string scanTopicName_;
	//! ROS subscribed topic queu size
	int scanTopicQueueSize_;
};

} /* namespace */
