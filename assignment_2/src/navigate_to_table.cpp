#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for sending goal to the move_base server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // Initialize the ROS node
    ros::init(argc, argv, "navigate_to_table");
    ros::NodeHandle nh;

    // Create a MoveBaseClient to send goals to the navigation stack
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to start...");
    ac.waitForServer(ros::Duration(5.0));
    ROS_INFO("Connected to move_base server.");

    // Define a goal
    move_base_msgs::MoveBaseGoal goal;

    // Set the goal parameters
    goal.target_pose.header.frame_id = "map";  // Goal relative to map frame
    goal.target_pose.header.stamp = ros::Time::now();

    // Set position and orientation in front of the pick-up table
    goal.target_pose.pose.position.x = 2.0;  // Adjust X position as needed
    goal.target_pose.pose.position.y = 0.0;  // Adjust Y position as needed
    goal.target_pose.pose.position.z = 0.0;  // Always 0 for 2D navigation

    // Orientation as quaternion
    goal.target_pose.pose.orientation.w = 1.0;  // Facing forward

    // Send the goal
    ROS_INFO("Sending goal to navigate in front of the pick-up table...");
    ac.sendGoal(goal);

    // Wait for result
    ac.waitForResult();

    // Check the result
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Successfully reached the pick-up table!");
    else
        ROS_ERROR("Failed to reach the pick-up table.");

    return 0;
}

