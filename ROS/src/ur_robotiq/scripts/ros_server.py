#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from ur_robotiq.msg import UR3eMoveitJoints, UR3eGripper
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface

def handle_arm_motion(msg: UR3eMoveitJoints, arm_planner: MoveGroupCommander):
    """
    Handle motion planning for the robotic arm using joint values from the message.
    """
    try:
        joint_goal = msg.joints  # in radians

        # Send the joint goal to MoveIt arm planner
        arm_planner.go(joint_goal, wait=True)
        arm_planner.stop()  # Ensure no residual motion

        rospy.loginfo("Arm successfully moved to the target positions.")

    except Exception as e:
        rospy.logerr(f"Failed to move arm: {e}")

def handle_gripper_motion(msg: UR3eGripper, gripper_planner: MoveGroupCommander):
    """
    Handle motion planning for the gripper using the value provided in the message.
    """
    try:
        gripper_value = msg.gripper

        # Validate the gripper value
        if gripper_value < 0 or gripper_value > 1:
            rospy.logerr("Invalid Gripper range")
            return

        # Map the gripper value (0 to 1) to the joint range (0.0 to 0.5523)
        gripper_position = gripper_value * 1  # Assuming this range for robotiq_85_left_knuckle_joint

        # Set the gripper joint target
        gripper_goal = {'robotiq_85_left_knuckle_joint': gripper_position}

        # Move the gripper to the desired position
        gripper_planner.set_joint_value_target(gripper_goal)
        gripper_planner.go(wait=True)
        gripper_planner.stop()  # Ensure no residual motion

        rospy.loginfo(f"Gripper successfully moved to position: {gripper_position:.4f}")

    except Exception as e:
        rospy.logerr(f"Failed to move gripper: {e}")

def moveit_planning_server():
    """
    Initialize the MoveIt planning server node and subscribe to the arm and gripper command topics.
    """
    rospy.init_node('moveit_planning_server', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize MoveIt components
    robot = RobotCommander() #Represents the entire robot.
    scene = PlanningSceneInterface() #Handles obstacles.
    arm = MoveGroupCommander('arm')  # Move group for the arm
    gripper = MoveGroupCommander('hand')  # Move group for the gripper

    # Subscribe to the topic publishing UR3eMoveitJoints messages for the arm
    rospy.Subscriber(
        '/ur3e_joints',
        UR3eMoveitJoints,
        lambda msg: handle_arm_motion(msg, arm)
    )

    # Subscribe to the topic publishing UR3eGripper messages for the gripper
    rospy.Subscriber(
        '/ur3e_gripper',
        UR3eGripper,
        lambda msg: handle_gripper_motion(msg, gripper)
    )

    rospy.loginfo("MoveIt Planning Server is ready and listening for joint commands.")
    rospy.spin()

if __name__ == '__main__':
    moveit_planning_server()
