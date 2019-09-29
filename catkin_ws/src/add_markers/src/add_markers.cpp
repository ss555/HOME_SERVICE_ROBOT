#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#define DIST_THRES 0.3f

//1st goal
#define goalX 4.0f
#define goalY 5.0f

#define goalX2 1.0f
#define goalY2 0.0f
float xRobo=0;
float yRobo=0;

enum markerState{
	PICK_UP=1,
	DROP_OFF=2,
};
markerState state=PICK_UP;
float isClose(){
	float dist=DIST_THRES;
	if(state==PICK_UP){
	float dx=goalX-xRobo;
	float dy=goalY-yRobo;
	dist=sqrtf(dx*dx+dy*dy);
	}else{
	float dx=goalX2-xRobo;
	float dy=goalY2-yRobo;
	dist=sqrtf(dx*dx+dy*dy);
	}
	return dist;
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	xRobo=msg->pose.pose.position.x;
	yRobo=msg->pose.pose.position.y;
	ROS_INFO("robot at x:%f, y:%f\n",xRobo,yRobo);
	//bool close=isClose();
	ROS_INFO("close: %f",isClose());
	//ROS_INFO("marker state:%d",markerState);
	//ROS_INFO("robot at x:");
	
}
int main( int argc, char** argv )
{
	// Create Node
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;

	// Create Publisher
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	
	//subscribe to the odometry
	ros::Subscriber subOdom=n.subscribe("/odom",10,odomCallback);
	// Create marker
	visualization_msgs::Marker marker;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "add_markers";
	marker.id = 0;

	// Set the marker type.  can be a CUBE, SPHERE, ARROW, or CYLINDER
	marker.type = visualization_msgs::Marker::CYLINDER;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	// Set the pickup pose of the marker.
	// This is relative to the frame/time specified above
	marker.pose.position.x = goalX;
	marker.pose.position.y = goalY;
	marker.pose.position.z = 0;

	// Set the orientation of the marker.
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;

	marker.lifetime = ros::Duration();
	
  // Check for subscribers
	while(marker_pub.getNumSubscribers()<1){
		ROS_WARN("Please subscribe to the marker");
		if(!ros::ok()){
			return 0;
		}
	}
	marker.action=visualization_msgs::Marker::ADD;
	marker_pub.publish(marker);
	ROS_INFO("Published marker at the pick_up point");
	ros::Rate r(1);
	bool picked=false;
	while(ros::ok()){
		bool close=(isClose()<DIST_THRES);
		
		if(close&&(!picked)){
			ROS_INFO("robot reached the pick up, 5 more seconds");
			marker.action=visualization_msgs::Marker::DELETE;
			
			marker_pub.publish(marker);
			state=DROP_OFF;
			picked=true;
			ros::Duration(5.0).sleep();
			
		}else if(close&&picked){
			ROS_INFO("robot reached the drop-off position");
			
			marker.pose.position.x=goalX2;
			marker.pose.position.y=goalY2;
			marker.action=visualization_msgs::Marker::ADD;
			
			marker_pub.publish(marker);
			ros::Duration(5.0).sleep();
			state=DROP_OFF;
			
		}else{
			//bool picked=false;
		}
		ROS_INFO("\n close:%d",close);
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("\n not shut down:%d",ros::ok());
		
}

