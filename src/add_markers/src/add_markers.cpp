#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"


// Pick-up an Drop-off points
float pick_up[2] = {1.0, -4.0};
float drop_off[2] = {-4.0, 5.0};

// Variable to keep the Robot's State
typedef enum _ROBOT_STATE {
		SEARCHING,
		PICKED_UP,
		DROPPED_OFF
} ROBOT_STATE;

// Variable to track / trigger Robot's actions
typedef enum _MARKER_ACTION {
		INIT,
		NONE,
		PICK_UP,
		DROP_OFF
} MARKER_ACTION;

// Initialize state variables
ROBOT_STATE robot_state = SEARCHING;
MARKER_ACTION marker_action = INIT;


void robotPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	//check if robot is in close proximity to pick-up or drop-off points
	bool picking_up = (abs(pick_up[0] - msg->pose.pose.position.x) < 0.005) && (abs(pick_up[1] - msg->pose.pose.position.y) < 0.005);
	bool dropping_off = (abs(drop_off[0] - msg->pose.pose.position.x) < 0.005) && (abs(drop_off[1] - msg->pose.pose.position.y) < 0.005);

	// if picking up or dropping off - check state. Set appropriate action and change state.
	if (picking_up && (robot_state == SEARCHING)) {
		marker_action = PICK_UP;
		robot_state = PICKED_UP;
		ROS_INFO("Picked-up marker");
	} 
	else if (dropping_off && (robot_state != DROPPED_OFF)) {
		marker_action = DROP_OFF;
		robot_state = DROPPED_OFF;
		ROS_INFO("Dropped-off marker");
	}

}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber robot_sub = n.subscribe("/odom",1000,robotPoseCallback);


	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

	
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "add_markers";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 1;
	marker.pose.position.y = -4;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	// Publish the marker
	while (marker_pub.getNumSubscribers() < 1)
	{
		if (!ros::ok())
		{
			return 0;
		}
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
	}
	

	while (ros::ok())
	{
		ros::spinOnce();

		// Display initial marker
		if (marker_action == INIT) {
			marker_pub.publish(marker);
			ROS_INFO("Show marker 1");
			marker_action = NONE;
		}

		// Pick-up the marker
		if (marker_action == PICK_UP) {
			ros::Duration(3.0).sleep();
			marker.action = visualization_msgs::Marker::DELETE;
			marker_pub.publish(marker);
			marker_action = NONE;

		}

		// Drop-ff the marker
		if (marker_action == DROP_OFF) {
			ros::Duration(3.0).sleep();
			marker.pose.position.x = -4;
			marker.pose.position.y = 5;
			marker.action = visualization_msgs::Marker::ADD;
			marker_pub.publish(marker);
			marker_action = NONE;
		}
	
		// Exit once dropped off
		if (robot_state == DROPPED_OFF && marker_action == NONE)
			return 0;

		r.sleep();
	}
}
