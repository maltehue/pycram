from threading import Lock
from typing import Any

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, QuaternionStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from ..external_interfaces.giskard import init_giskard_interface
from ..external_interfaces.robokudo import query, queryEmpty
from ..helper import _apply_ik
from ..external_interfaces import giskard
from .default_process_modules import *
from scipy.spatial import distance


class StretchNavigate(DefaultNavigation):
    """
    Process module for the simulated Stretch that sends a cartesian goal to the robot to move the robot base
    """
    pass


class StretchMoveHead(ProcessModule):
    """
    Process module for the simulated Stretch that moves the head such that it looks at the given position
    """

    def _execute(self, designator) -> Any:
        target = designator.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2) * -1

        current_pan = robot.get_joint_state("joint_head_pan")
        current_tilt = robot.get_joint_state("joint_head_tilt")

        robot.set_joint_state("joint_head_pan", new_pan + current_pan)
        robot.set_joint_state("joint_head_tilt", new_tilt + current_tilt)


class StretchMoveGripper(DefaultMoveGripper):
    """
    Process module for the simulated Stretch that opens or closes the gripper
    """
    pass


class StretchDetecting(DefaultDetecting):
    """
    Process Module for the simulated Stretch that tries to detect an object fitting the given object description
    """
    pass


class StretchMoveTCP(DefaultMoveTCP):
    """
    Process module for the simulated Stretch that moves the tool center point of the robot
    """
    pass


class StretchMoveArmJoints(DefaultMoveArmJoints):
    """
    Process module for the simulated Stretch that moves the arm joints of the robot
    """
    pass


class StretchMoveJoints(DefaultMoveJoints):
    """
    Process module for the simulated Stretch that moves any joint of the robot
    """
    pass


class StretchWorldStateDetecting(DefaultWorldStateDetecting):
    """
    Process Module for the simulated Stretch that tries to detect an object using the world state
    """
    pass


class StretchOpen(ProcessModule):
    """
    Process module for the simulated Stretch that opens an already grasped container
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1] - 0.05)


class StretchClose(ProcessModule):
    """
    Process module for the simulated Stretch that closes an already grasped container
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)


###########################################################
########## Process Modules for the Real Stretch ###########
###########################################################


class StretchNavigationReal(ProcessModule):
    """
    Process module for the real Stretch that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, desig):
        # rospy.logdebug(f"Sending goal to giskard to Move the robot")
        # giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        # target_pose = giskard._pose_to_pose_stamped(designator.target)
        # giskard.giskard_wrapper.set_diff_drive_base_goal(target_pose, robot_description.base_link, "map")
        # giskard.giskard_wrapper.execute()

        def active_callback():
            rospy.loginfo("Start Navigating")

        def feedback_callback(msg):
            pass

        def done_callback(state, result):
            rospy.loginfo("Finished Navigation")
            for k in GoalStatus.__dict__.keys():
                if state == GoalStatus.__dict__[k]:
                    rospy.loginfo(f"Navigation has Finished with the result: {k}")

        goal = MoveBaseGoal()
        pose = desig.target
        goal.target_pose.pose = pose

        goal.target_pose.header.frame_id = "map"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(goal, active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        client.wait_for_result()


@init_giskard_interface
class StretchMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))
        goal_point = local_transformer.transform_pose(target, 'map')

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2) * -1

        current_pan = robot.get_joint_state("joint_head_pan")
        current_tilt = robot.get_joint_state("joint_head_tilt")

        # giskard.avoid_all_collisions()
        giskard.giskard_wrapper.allow_all_collisions()
        # giskard.achieve_joint_goal({"joint_head_pan": new_pan + current_pan,
        #                             "joint_head_tilt": new_tilt + current_tilt})
        from geometry_msgs.msg import Vector3Stamped, PointStamped
        pointing_axis = Vector3Stamped()
        pointing_axis.header.frame_id = 'link_head_tilt'
        pointing_axis.vector.x = 1
        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.point = goal_point.pose.position
        giskard.giskard_wrapper.set_pointing_goal(goal_point=goal,
                                                  tip_link='link_head_tilt',
                                                  pointing_axis=pointing_axis,
                                                  root_link='link_head')
        giskard.giskard_wrapper.execute()


class StretchDetectingReal(ProcessModule):
    """
    Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query(ObjectDesignatorDescription(types=[designator.object_type]))
        lt = LocalTransformer()
        robot_position = (lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y)
        idx = 0
        for i in range(0, len(query_result)):
            position_i = (query_result[i].pose.position.x, query_result[i].pose.position.y)
            position_idx = (query_result[idx].pose.position.x, query_result[idx].pose.position.y)
            if distance.euclidean(robot_position, position_i) < distance.euclidean(robot_position, position_idx):
                idx = i
        obj_pose = Pose.from_pose_stamped(query_result[idx])
        # lt = LocalTransformer()
        # obj_pose = lt.transform_pose(obj_pose, BulletWorld.robot.get_link_tf_frame("camera_color_optical_frame"))
        # obj_pose.orientation = [0, 0, 0, 1]
        # obj_pose.position.x -= 0.1

        bullet_obj = BulletWorld.current_bullet_world.get_objects_by_type(designator.object_type)
        if bullet_obj:
            bullet_obj[0].set_pose(obj_pose)
            return bullet_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)

            return bowl
        elif designator.object_type == ObjectType.SPOON:
            spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=obj_pose)
            return spoon
        elif designator.object_type == ObjectType.YELLOW_CUP:
            yellow_cup = Object("yellow_cup", ObjectType.YELLOW_CUP, "jeroen_cup.stl", pose=obj_pose)
            return yellow_cup

        return bullet_obj[0]


@init_giskard_interface
class StretchMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real Stretch while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        if designator.allow_gripper_collision:
            pass
            # giskard.allow_gripper_collision(designator.arm)
            # giskard.allow_self_collision()

        # giskard.giskard_wrapper.avoid_self_collision()

        giskard.giskard_wrapper.allow_all_collisions()
        q = QuaternionStamped()
        q.header.frame_id = 'map'
        q.quaternion = lt.bullet_world.robot.pose.orientation

        giskard.giskard_wrapper.set_rotation_goal(q, tip_link='base_link',
                                                  root_link='map', add_monitor=False)

        giskard.achieve_straight_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                                'map')



@init_giskard_interface
class StretchMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real Stretch to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        giskard.giskard_wrapper.allow_all_collisions()
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        # giskard.avoid_all_collisions()
        giskard.allow_self_collision()
        giskard.achieve_joint_goal(joint_goals)


@init_giskard_interface
class StretchMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.giskard_wrapper.allow_all_collisions()
        # giskard.avoid_all_collisions()
        # giskard.allow_self_collision()
        giskard.achieve_joint_goal(name_to_position)


@init_giskard_interface
class StretchMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        chain = robot_description.chains[designator.gripper].gripper.get_static_joint_chain(designator.motion)
        giskard.giskard_wrapper.allow_all_collisions()
        giskard.achieve_joint_goal(chain)


class StretchOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(robot_description.get_tool_frame(designator.arm),
                                            designator.object_part.name)


class StretchCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(robot_description.get_tool_frame(designator.arm),
                                             designator.object_part.name)


class StretchManager(ProcessModuleManager):
    def __init__(self):
        super().__init__("stretch")
        self._navigate_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchNavigate(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return StretchWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchCloseReal(self._close_lock)
