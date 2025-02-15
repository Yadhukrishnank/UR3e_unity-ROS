#!/usr/bin/env python3
from __future__ import print_function

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur_robotiq.srv import UrMoverService, UrMoverServiceRequest, UrMoverServiceResponse

# Define the joint names for the UR3e robot
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, start_joint_angles)
        raise Exception(exception_str)

    return planCompat(plan)

def plan_move_above(req):
    response = UrMoverServiceResponse()

    rospy.loginfo("Received request with joints: %s", req.joints_input.joints)
    rospy.loginfo("Received pick_pose: position=%s, orientation=%s", 
                  req.pick_pose.position, req.pick_pose.orientation)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints_input.joints

    # Define sideways offsets (adjust these values as needed)
    sideways_offsets = [0.05, -0.05, 0.10, -0.10]  # X-axis adjustments

    planning_succeeded = False
    target_pose = None
    pre_grasp_pose = None

    # Try original plan (directly above the pick_pose)
    try:
        pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
        target_pose = req.pick_pose
        planning_succeeded = True
        rospy.loginfo("Direct plan succeeded.")
    except Exception as e:
        rospy.logwarn("Direct plan failed: %s. Trying sideways offsets...", str(e))

    # If direct plan failed, try sideways offsets
    if not planning_succeeded:
        for offset in sideways_offsets:
            try:
                adjusted_pick_pose = copy.deepcopy(req.pick_pose)
                adjusted_pick_pose.position.x += offset  # Adjust X-axis
                rospy.loginfo("Attempting sideways offset: %s m (X-axis)", offset)

                pre_grasp_pose = plan_trajectory(move_group, adjusted_pick_pose, current_robot_joint_configuration)
                target_pose = adjusted_pick_pose
                planning_succeeded = True
                rospy.loginfo("Sideways plan succeeded with offset: %s m", offset)
                break  # Exit loop on success
            except Exception as e:
                rospy.logwarn("Sideways plan with offset %s failed: %s", offset, str(e))

    # If no plan found, return error
    if not planning_succeeded:
        rospy.logerr("All planning attempts failed.")
        return response

    # Plan grasp motion (lower Z from the target_pose)
    try:
        grasp_pose = copy.deepcopy(target_pose)
        grasp_pose.position.z -= 0.05  # Lower gripper by 5 cm

        # Get final joint angles from pre-grasp trajectory
        pre_grasp_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
        grasp_trajectory = plan_trajectory(move_group, grasp_pose, pre_grasp_joint_angles)

    except Exception as e:
        rospy.logerr("Grasp trajectory planning failed: %s", str(e))
        return response

    # Validate grasp trajectory
    if not grasp_trajectory.joint_trajectory.points:
        rospy.logwarn("Grasp trajectory has no valid points.")
        return response

    # Add trajectories to response
    response.trajectories.append(pre_grasp_pose)
    response.trajectories.append(grasp_trajectory)
    rospy.loginfo("Trajectories successfully planned.")

    move_group.clear_pose_targets()
    return response

def ground_collision():
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    ground_pose = geometry_msgs.msg.PoseStamped()
    ground_pose.header.frame_id = "world"
    ground_pose.pose.position.z = -0.072
    scene.add_box("ground", ground_pose, size=(2, 2, 0.02))
    rospy.loginfo("Ground collision object added.")

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur3e_moveit_server')

    ground_collision()

    s = rospy.Service('ur3e_moveit', UrMoverService, plan_move_above)
    rospy.loginfo("Ready to plan trajectories.")
    rospy.spin()

if __name__ == "__main__":
    moveit_server()
    

# from __future__ import print_function

# import rospy
# import sys
# import copy
# import moveit_commander
# import moveit_msgs.msg
# from moveit_msgs.msg import RobotState
# from sensor_msgs.msg import JointState
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
# import tf.transformations

# from ur_robotiq.srv import UrMoverService, UrMoverServiceRequest, UrMoverServiceResponse

# # Define the joint names for the UR3e robot
# joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
#                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# if sys.version_info >= (3, 0):
#     def planCompat(plan):
#         return plan[1]
# else:
#     def planCompat(plan):
#         return plan

# def plan_trajectory(move_group, destination_pose, start_joint_angles): 
#     current_joint_state = JointState()
#     current_joint_state.name = joint_names
#     current_joint_state.position = start_joint_angles

#     moveit_robot_state = RobotState()
#     moveit_robot_state.joint_state = current_joint_state
#     move_group.set_start_state(moveit_robot_state)

#     move_group.set_pose_target(destination_pose)
#     plan = move_group.plan()

#     if not plan:
#         exception_str = """
#             Trajectory could not be planned for a destination of {} with starting joint angles {}.
#             Please make sure target and destination are reachable by the robot.
#         """.format(destination_pose, start_joint_angles)
#         raise Exception(exception_str)

#     return planCompat(plan)

# def align_end_effector(pick_pose):
#     quat = [pick_pose.orientation.x, pick_pose.orientation.y, 
#             pick_pose.orientation.z, pick_pose.orientation.w]
#     rpy = tf.transformations.euler_from_quaternion(quat)
    
#     aligned_pose = copy.deepcopy(pick_pose)
#     aligned_quat = tf.transformations.quaternion_from_euler(0, 3.14, rpy[2])  # Flip wrist
#     aligned_pose.orientation.x = aligned_quat[0]
#     aligned_pose.orientation.y = aligned_quat[1]
#     aligned_pose.orientation.z = aligned_quat[2]
#     aligned_pose.orientation.w = aligned_quat[3]
    
#     return aligned_pose

# def plan_move_above(req):
#     response = UrMoverServiceResponse()

#     rospy.loginfo("Received request with joints: %s", req.joints_input.joints)
#     rospy.loginfo("Received pick_pose: position=%s, orientation=%s", 
#                   req.pick_pose.position, req.pick_pose.orientation)

#     group_name = "arm"
#     move_group = moveit_commander.MoveGroupCommander(group_name)

#     current_robot_joint_configuration = req.joints_input.joints

#     # Align the end-effector before planning
#     aligned_pick_pose = align_end_effector(req.pick_pose)

#     # Define sideways offsets (adjust these values as needed)
#     sideways_offsets = [0.05, -0.05, 0.10, -0.10]  # X-axis adjustments

#     planning_succeeded = False
#     target_pose = None
#     pre_grasp_pose = None

#     # Try original plan (directly above the pick_pose)
#     try:
#         pre_grasp_pose = plan_trajectory(move_group, aligned_pick_pose, current_robot_joint_configuration)
#         target_pose = aligned_pick_pose
#         planning_succeeded = True
#         rospy.loginfo("Direct plan succeeded.")
#     except Exception as e:
#         rospy.logwarn("Direct plan failed: %s. Trying sideways offsets...", str(e))

#     # If direct plan failed, try sideways offsets
#     if not planning_succeeded:
#         for offset in sideways_offsets:
#             try:
#                 adjusted_pick_pose = copy.deepcopy(aligned_pick_pose)
#                 adjusted_pick_pose.position.x += offset  # Adjust X-axis
#                 rospy.loginfo("Attempting sideways offset: %s m (X-axis)", offset)

#                 pre_grasp_pose = plan_trajectory(move_group, adjusted_pick_pose, current_robot_joint_configuration)
#                 target_pose = adjusted_pick_pose
#                 planning_succeeded = True
#                 rospy.loginfo("Sideways plan succeeded with offset: %s m", offset)
#                 break  # Exit loop on success
#             except Exception as e:
#                 rospy.logwarn("Sideways plan with offset %s failed: %s", offset, str(e))

#     # If no plan found, return error
#     if not planning_succeeded:
#         rospy.logerr("All planning attempts failed.")
#         return response

#     # Plan grasp motion (lower Z from the target_pose)
#     try:
#         grasp_pose = copy.deepcopy(target_pose)
#         grasp_pose.position.z -= 0.05  # Lower gripper by 5 cm

#         # Get final joint angles from pre-grasp trajectory
#         pre_grasp_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
#         grasp_trajectory = plan_trajectory(move_group, grasp_pose, pre_grasp_joint_angles)

#     except Exception as e:
#         rospy.logerr("Grasp trajectory planning failed: %s", str(e))
#         return response

#     # Validate grasp trajectory
#     if not grasp_trajectory.joint_trajectory.points:
#         rospy.logwarn("Grasp trajectory has no valid points.")
#         return response

#     # Add trajectories to response
#     response.trajectories.append(pre_grasp_pose)
#     response.trajectories.append(grasp_trajectory)
#     rospy.loginfo("Trajectories successfully planned.")

#     move_group.clear_pose_targets()
#     return response

# def ground_collision():
#     scene = moveit_commander.PlanningSceneInterface()
#     rospy.sleep(1)

#     ground_pose = geometry_msgs.msg.PoseStamped()
#     ground_pose.header.frame_id = "world"
#     ground_pose.pose.position.z = -0.02
#     scene.add_box("ground", ground_pose, size=(2, 2, 0.02))
#     rospy.loginfo("Ground collision object added.")

# def moveit_server():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('ur3e_moveit_server')

#     ground_collision()

#     s = rospy.Service('ur3e_moveit', UrMoverService, plan_move_above)
#     rospy.loginfo("Ready to plan trajectories.")
#     rospy.spin()

# if __name__ == "__main__":
#     moveit_server()
