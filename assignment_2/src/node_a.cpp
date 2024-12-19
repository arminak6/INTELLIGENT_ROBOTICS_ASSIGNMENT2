#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <std_srvs/Empty.h>
#include "assignment_2/Coeffs.h" 

int main(int argc, char** argv) {
    ros::init(argc, argv, "placing_routine_node_a");
    ros::NodeHandle nh;

    // Service client to request coefficients from /straight_line_srv
    ros::ServiceClient client = nh.serviceClient<assignment_2::Coeffs>("/straight_line_srv");
    assignment_2::Coeffs srv;

    // TF buffer and listener to handle coordinate transformation
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Target position on the line
    double x = 0.1; // Choose a suitable X-coordinate
    double y, m, q;

    ROS_INFO("Requesting line coefficients from /straight_line_srv...");
    if (client.call(srv)) {
        m = srv.response.m;
        q = srv.response.q;
        ROS_INFO("Received coefficients: m = %f, q = %f", m, q);

        // Calculate the corresponding y-coordinate for the chosen x
        y = m * x + q;
        ROS_INFO("Calculated target point on the line: x = %f, y = %f", x, y);
    } else {
        ROS_ERROR("Failed to call /straight_line_srv");
        return 1;
    }

    // Define the pose in the AprilTag reference frame
    geometry_msgs::PoseStamped target_pose_in_apriltag;
    target_pose_in_apriltag.header.frame_id = "apriltag_10"; // Reference frame of AprilTag ID 10
    target_pose_in_apriltag.header.stamp = ros::Time::now();

    target_pose_in_apriltag.pose.position.x = x;
    target_pose_in_apriltag.pose.position.y = y;
    target_pose_in_apriltag.pose.position.z = 0.0;
    target_pose_in_apriltag.pose.orientation.w = 1.0;

    try {
        // Transform the pose from the AprilTag frame to the robot's base frame
        geometry_msgs::PoseStamped target_pose_in_base;
        tfBuffer.transform(target_pose_in_apriltag, target_pose_in_base, "base_link");

        ROS_INFO("Target Pose in Robot Base Frame:");
        ROS_INFO("Position - x: %f, y: %f, z: %f",
                 target_pose_in_base.pose.position.x,
                 target_pose_in_base.pose.position.y,
                 target_pose_in_base.pose.position.z);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("TF Transformation Error: %s", ex.what());
        return 1;
    }

    ros::spinOnce();
    return 0;
}

