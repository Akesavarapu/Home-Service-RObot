#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define PICK_UP_X -2.0
#define PICK_UP_Y 5.0
#define DROP_OFF_X 5
#define DROP_OFF_Y -2
#define W 1.5
#define threshold 0.05

class add_markers{

	private:
	    ros::NodeHandle n;
	    ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 10, &add_markers::amcl_callback, this);
	    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	    visualization_msgs::Marker marker;
	    geometry_msgs::Pose amcl_pose;

	    bool picked = false;
	    bool dropped = false;


	public:
	    add_markers(ros::NodeHandle& n) : n(n) {

		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();

		marker.ns = "add_markers";
		marker.id = 0;

		uint32_t shape = visualization_msgs::Marker::SPHERE;
		marker.type = shape;

		marker.action = visualization_msgs::Marker::ADD;

		// pose
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0; 
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		// scale
		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;

		// color
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		while(ros::ok())
		{   
		    if (dropped) break;
		    get_nav_state();
		}
	    }

	    ~add_markers() {};

	    void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose_msg) {

		amcl_pose = amcl_pose_msg.pose.pose;
	    }


	    void get_nav_state() {

		double x = amcl_pose.position.x, y = amcl_pose.position.y, w = amcl_pose.orientation.w;

		while (marker_pub.getNumSubscribers() < 1) {
		    if (!ros::ok()) return;

		    ROS_WARN_ONCE("Please create a subscriber to the marker!");
		    sleep(1);
		}

		// not yet reached pick up spot
		if (!picked)  {
		    // Set Marker
		    marker.pose.position.x = PICK_UP_X;
		    marker.pose.position.y = PICK_UP_Y;
		    marker.pose.orientation.w = W;
		    marker_pub.publish(marker);

		    // Within threshold of pick up spot
		    if(abs(PICK_UP_X - x) < threshold && abs(PICK_UP_Y - y) < threshold) {
			ROS_INFO("hello you have reached pick up zone");
			picked = true;
			marker.color.a = 0.0;
			marker_pub.publish(marker);
			ros::Duration(5.0).sleep();
		    }
		}

		// Not yet reached drop off spot
		else if(!dropped) {
		    // Within threshold of drop off spot
		    if(abs(DROP_OFF_X - x) <  threshold  && abs(DROP_OFF_Y -  y) <  threshold) {
			ROS_INFO("hello you have reached drop off zone");
			dropped = true;
			marker.color.a = 1.0;
			marker.pose.position.x = DROP_OFF_X;
			marker.pose.position.y = DROP_OFF_Y;
			marker.pose.orientation.w = W;
			marker_pub.publish(marker);	
			ros::Duration(5.0).sleep();
		    }
		}

		ros::spinOnce();
	    }
};

int main( int argc, char** argv )
{   
    
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(20);
    add_markers add_markers(n);

    return 0;
}
