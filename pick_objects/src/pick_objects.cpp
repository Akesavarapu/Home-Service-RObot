#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the "move_base" server,
// through the "SimpleActionClient"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_objects");

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // FIRST GOAL
    goal.target_pose.pose.position.x = -2.0;
    goal.target_pose.pose.position.y = 5.0;
    goal.target_pose.pose.orientation.w  = 1.0;

    ROS_INFO("Sending goal #1");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The base moved 1 meter forward");
    } else {
        ROS_INFO("ERROR: The base failed to move forward 1 meter");
    }

    ros::Duration(5.0).sleep();
    ROS_INFO("Waited for 5 seconds");

    // SECOND GOAL
    goal.target_pose.pose.position.x = 5;
    goal.target_pose.pose.position.y = -2;
    goal.target_pose.pose.orientation.w =  1.0;
   
    ROS_INFO("Sending goal #2");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The base moved 1 meter forward");
    } else {
        ROS_INFO("ERROR: The base failed to move forward 1 meter");
    }

    

    return 0;

}
