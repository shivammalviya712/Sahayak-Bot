'''
Manipulator Module
Author: eYRC_SB_363
'''

import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, Grasp, PlaceLocation


class Manipulator:
  """"""

  def __init__(self):
    # Robot outer level interface
    self.robot = moveit_commander.RobotCommander()

    # Scene Interface
    self.scene = moveit_commander.PlanningSceneInterface()

    # Planning groups
    arm_group_name = 'arm'
    self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
    # gripper_group_name = 'gripper'
    # self.gripper_group = moveit_commander.MoveGroupCommander(
    #     gripper_group_name)

    # Publish trajectories for RViz to visualize
    self.display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        DisplayTrajectory,
        queue_size=20
    )

    # Check
    rospy.loginfo('Robot group name: ' + str(self.robot.get_group_names()))
    rospy.loginfo('Planning scene objects: ' +
                  str(self.scene.get_known_object_names()))
    rospy.loginfo('Arm end effector: ' +
                  str(self.arm_group.get_end_effector_link()))
    # rospy.loginfo('Gripper has end effector: ' +
    #               str(self.gripper_group.has_end_effector_link()))
    rospy.loginfo("Robot.get_current_state(): " +
                  str(self.robot.get_current_state()))
    # rospy.loginfo("gripper_group.get_current_goint_values(): " +
    #               str(self.gripper_group.get_current_joint_values()))
    rospy.loginfo("arm_group.get_current_goint_values(): " +
                  str(self.arm_group.get_current_joint_values()))
    rospy.loginfo('Setup complete')

  def openGripper(self, posture):
    """
    Set the open gripper pose
    Parameters
    ----------
    posture : trajectory_msgs.msg.JointTrajectory
                    Gripper posture.
    """
    posture.joint_names = [str for i in range(1)]
    posture.joint_names[0] = "gripper_finger1_joint"

    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(1)]
    posture.points[0].positions[0] = 0.01
    posture.points[0].time_from_start = rospy.Duration(0.5)

  def closeGripper(self, posture, gripper_close):
    """
    Set the open gripper pose
    Parameters
    ----------
    posture : trajectory_msgs.msg.JointTrajectory
                    Gripper posture.
    """
    posture.joint_names = [str for i in range(1)]
    posture.joint_names[0] = "gripper_finger1_joint"

    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(1)]
    posture.points[0].positions[0] = gripper_close
    # TODO: Play with this
    posture.points[0].time_from_start = rospy.Duration(0.5)

  def place(self, object_config):
    """
    > Place object

    Parameters
    ----------
    object_config: Config of the object to be placed
    """

    place_location = PlaceLocation()

    # The position of the end-effector for the place relative to a reference frame
    place_location.place_pose.header.frame_id = "world"
    place_location.place_pose.pose = object_config.place_pose

    # The approach motion
    place_location.pre_place_approach.direction.header.frame_id = "world"
    place_location.pre_place_approach.direction.vector.z = -1.0
    place_location.pre_place_approach.min_distance = 0.1
    place_location.pre_place_approach.desired_distance = 0.115

    # The retreat motion
    place_location.post_place_retreat.direction.header.frame_id = "world"
    place_location.post_place_retreat.direction.vector.z = 1.0
    place_location.post_place_retreat.min_distance = 0.1
    place_location.post_place_retreat.desired_distance = 0.115

    # Set open gripper pose after reaching the place position
    self.openGripper(place_location.post_place_posture)

    # For pick/place operations, the name of the support surface is used
    # to specify the fact that attached objects are allowed to touch the
    # support surface.
    self.arm_group.set_support_surface_name(object_config.support_surface)

    # Call place to place the object using the place locations given
    self.arm_group.place(object_config.object_id, place_location)

  def pick(self, object_config):
    """
    > Pick object

    Parameters
    ----------
    object_config: Config of the object to be pick
    """
    grasp = Grasp()

    # The pose of the end-effector for the grasp
    # with reference coordinate frame and timestamp.
    # This is the pose of the "parent_link" of the
    # end-effector, not actually the pose of any
    # link *in* the end-effector.
    grasp.grasp_pose.header.frame_id = "world"
    grasp.grasp_pose.pose = object_config.pick_pose

    # The approach direction to take before picking an object.
    grasp.pre_grasp_approach.direction.header.frame_id = "world"
    grasp.pre_grasp_approach.direction.vector.z = -1.0
    # The min distance that must be considered feasible before the
    # grasp is even attempted
    grasp.pre_grasp_approach.min_distance = 0.095
    # The desired translation distance
    grasp.pre_grasp_approach.desired_distance = 0.115

    # The retreat direction to take after a grasp has been completed (object is attached)
    grasp.post_grasp_retreat.direction.header.frame_id = "world"
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    # The min distance that must be considered feasible before the
    # grasp is even attempted
    grasp.post_grasp_retreat.min_distance = 0.1
    # The desired translation distance
    grasp.post_grasp_retreat.desired_distance = 0.25

    # The internal posture of the hand for the pre-grasp
    # only positions are used
    self.openGripper(grasp.pre_grasp_posture)

    # The internal posture of the hand for the grasp
    # positions and efforts are used
    self.closeGripper(grasp.grasp_posture, object_config.gripper_close)

    # For pick/place operations, the name of the support surface is used
    # to specify the fact that attached objects are allowed to touch the
    # support surface.
    self.arm_group.set_support_surface_name('short_table.dae')

    # Finally the most awaited moment, now pick it
    self.arm_group.pick(object_config.object_id, grasp)
