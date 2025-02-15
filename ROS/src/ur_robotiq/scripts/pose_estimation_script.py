#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from math import pi

def move_to_pose(target_position, target_orientation):
    # Initialize MoveIt! and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_pose', anonymous=True)

    # Create a MoveGroupCommander object for the robot arm
    group_name = "arm"  # Replace with your robot's planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the planning time (in seconds)
    move_group.set_planning_time(10.0)

    # Define the target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = target_position[0]
    target_pose.position.y = target_position[1]
    target_pose.position.z = target_position[2]
    target_pose.orientation.x = target_orientation[0]
    target_pose.orientation.y = target_orientation[1]
    target_pose.orientation.z = target_orientation[2]
    target_pose.orientation.w = target_orientation[3]

    # Set the target pose
    move_group.set_pose_target(target_pose)

    # Plan the trajectory
    rospy.loginfo("Planning trajectory to target pose...")
    plan = move_group.plan()

    # Check if planning was successful
    if not plan:
        rospy.logerr("Failed to plan trajectory to target pose.")
        return False

    # Execute the planned trajectory
    rospy.loginfo("Executing trajectory...")
    move_group.go(wait=True)

    # Clear the pose target
    move_group.clear_pose_targets()

    rospy.loginfo("Trajectory execution complete.")
    return True

if __name__ == "__main__":
    try:
        # Define the target position (x, y, z) in meters
        target_position = [0.0005, 0.0001, 0.0003]  # Example position

        # Define the target orientation (x, y, z, w) as a quaternion
        target_orientation = [0.0, 0.0, 0.0, -0.5]  # Example orientation (no rotation)

        # Move to the target pose
        success = move_to_pose(target_position, target_orientation)

        if success:
            rospy.loginfo("Move to pose succeeded!")
        else:
            rospy.logerr("Move to pose failed.")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        # Shutdown MoveIt! and ROS
        moveit_commander.roscpp_shutdown()



