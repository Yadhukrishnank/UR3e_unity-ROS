#!/usr/bin/env python3
import rospy
from ur_robotiq.srv import UrMoverService, UrMoverServiceResponse
from moveit_msgs.msg import RobotTrajectory

def callback(request):
    """
    Callback function for the UR3e MoveIt service.
    Processes the request and returns a response.
    """
    rospy.loginfo("Received request:")
    rospy.loginfo(f"Joint Input: {request.joints_input.joints}")
    rospy.loginfo(f"Pick Pose: {request.pick_pose}")
    rospy.loginfo(f"Place Pose: {request.place_pose}")

    # Create an empty response
    response = UrMoverServiceResponse()

    # Simulate a trajectory (you should replace this with actual MoveIt planning)
    dummy_trajectory = RobotTrajectory()
    response.trajectories.append(dummy_trajectory)  # Add a trajectory to response

    rospy.loginfo("Returning a dummy trajectory response.")
    return response  # Return the response with the trajectory

def moveit_service_server():
    """Initialize the MoveIt service server."""
    rospy.init_node("ur3e_moveit_server")
    rospy.Service("/ur3e_moveit_service", UrMoverService, callback)
    rospy.loginfo("UR3e MoveIt Service is ready.")
    rospy.spin()

if __name__ == "__main__":
    moveit_service_server()
