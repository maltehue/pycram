import rospy

from .default_process_modules import *
from ..datastructures.enums import JointType
from ..datastructures.world import World
from ..designators.motion_designator import *
from ..external_interfaces.ik import request_giskard_ik
from ..external_interfaces.robokudo import *
from ..robot_description import RobotDescription
from pycrap.ontologies import *
from pycram.datastructures.pose import Transform
from ..datastructures.pose import Pose
from ..external_interfaces.giskard import init_giskard_interface
from ..external_interfaces.robokudo import query_object
from ..world_concepts import world_object
from ..worlds.bullet_world import BulletWorld
from ..utils import _apply_ik
from geometry_msgs.msg import Quaternion, QuaternionStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
from actionlib_msgs.msg import GoalStatus


class StretchMoveHead(ProcessModule):
    """
    Process module for the simulated Stretch that moves the head such that it looks at the given position
    """

    def _execute(self, designator: MoveMotion) -> Any:
        target = designator.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        current_pan = robot.get_joint_position("joint_head_pan")

        robot.set_joint_position("joint_head_pan", new_pan + current_pan)

        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              np.sqrt(pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2)) * -1
        current_tilt = robot.get_joint_position("joint_head_tilt")

        robot.set_joint_position("joint_head_tilt", new_tilt + current_tilt)

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
    gripper = RobotDescription.get_tool_frame(arm)

    joints = RobotDescription.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)

###########################################################
########## Process Modules for the Real Stretch ###########
###########################################################

@init_giskard_interface
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
        #is this how it should be defined??
        robot = world_object.World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)

        current_pan = robot.get_joint_position("joint_head_pan")
        current_tilt = robot.get_joint_position("joint_head_tilt")

        # giskard.avoid_all_collisions()
        # giskard.achieve_joint_goal({"joint_head_pan": new_pan + current_pan})

        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              np.sqrt(pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2)) * -1
        current_tilt = robot.get_joint_position("joint_head_tilt")
        # giskard.avoid_all_collisions()
        giskard.giskard_wrapper.allow_all_collisions()
        giskard.achieve_joint_goal({"joint_head_tilt": new_tilt + current_tilt,
                                    "joint_head_pan": new_pan + current_pan})


@init_giskard_interface
class StretchDetectingReal(ProcessModule):
    """
    Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:

        color=ObjectDesignatorDescription(types=[designator.object_designator_description.obj_type]).resolve().color
        #print("color ",color)
        query_result = query_object(ObjectDesignatorDescription(types=[designator.object_designator_description.obj_type]),color)
        obj_pose = query_result.res[0].pose[1]
        obj_pose=PoseStamped.from_ros_message(obj_pose)
        #print(f"obj_pose: {obj_pose}")
        lt = LocalTransformer()
        obj_pose.header.frame_id = world_object.World.robot.get_link_tf_frame(obj_pose.header.frame_id)
        obj_pose = lt.transform_pose(obj_pose, "map")
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        bullet_obj = world_object.World.current_world.get_object_by_type(designator.object_designator_description.obj_type)
        if bullet_obj:
            bullet_obj[0].set_pose(obj_pose)
            return bullet_obj[0]
        elif designator.object_designator_description.obj_type == Cup:
            cup = Object("cup", Cup, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_designator_description.obj_type == Bowl:
            bowl = Object("bowl", Bowl, "bowl.stl", pose=obj_pose)
            return bowl

        return bullet_obj[0]

@init_giskard_interface
class StretchMoveTCPReal(ProcessModule):
    def _execute(self, designator: MoveTCPMotion):
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        giskard.allow_gripper_collision(designator.arm)

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        #print("POSE IN MAP", pose_in_map)
        giskard.giskard_wrapper.allow_self_collision()
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(),
                                       "map")




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
        chain = RobotDescription.current_robot_description.get_arm_chain(designator.gripper).get_static_gripper_state(
            designator.motion)
        giskard.achieve_joint_goal(chain)


class StretchOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(RobotDescription.get_tool_frame(designator.arm),
                                            designator.object_part.name)


class StretchCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(RobotDescription.get_tool_frame(designator.arm),
                                             designator.object_part.name)

class StretchManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "stretch_description"
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
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchDetectingReal(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return StretchOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return StretchClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchCloseReal(self._close_lock)