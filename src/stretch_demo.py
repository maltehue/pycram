from pycram.bullet_world import *
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.designators.motion_designator import *
from pycram.process_module import real_robot
from pycram.designators.action_designator import *
from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
import traceback
import faulthandler

world = BulletWorld()
stretch = Object("stretch", ObjectType.ROBOT, "stretch.urdf", ignoreCachedFiles=True)
stretch_designator = ObjectDesignatorDescription(names=['stretch']).resolve()

r = RobotStateUpdater("/tf", "/joint_states")
lt = LocalTransformer()

table = Object("table", ObjectType.GENERIC_OBJECT, "big_table.stl", pose=Pose([14.55, 1.05, 0.35], [0, 0, 0, 1]))
coffee_table = Object("cofee_table", ObjectType.GENERIC_OBJECT, "coffee_table.stl",
                      pose=Pose([12.75, 3.55, 0], [0, 0, 1, 1]))
armchair = Object("armchair", ObjectType.GENERIC_OBJECT, "armchair_lowres.stl",
                  pose=Pose([14.1, 3.45, 0.355], [0, 0, -1, 1]))
sofa = Object("sofa", ObjectType.GENERIC_OBJECT, "sofa_lowres.stl", pose=Pose([12.8, 4.75, 0.355], [0, 0, 0, 1]))

# Change coordinates according to new environment if needed
looking_cup = LookAtAction([Pose([14.8, 0.8, 0.9])])
looking = LookAtAction([Pose([13.8, 0.8, 0.86])])
looking_spoon = LookAtAction([Pose([14.0, 0.8, 0.83])])
looking_spoon_table_2 = LookAtAction([Pose([14.9, 0.8, 0.7])])

retrieve_arm_fully = MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0",
                                       "joint_lift"],
                                      [0., 0., 0., 0., 1.1])
open_gripper = SetGripperAction(["arm"], ["open"])
close_gripper = SetGripperAction(["arm"], ["close"])
park_arms = ParkArmsAction([Arms.BOTH])

# Change coordinates according to new environment
# Robot navigates closer and more to the front when picking up spoon for better perception
navigate_to_table_1 = NavigateAction([Pose([13.6, 1.55, 0.0], [0.000, 0.000, 0.0, 1.000])])
navigate_to_table_1_spoon = NavigateAction([Pose([13.65, 1.4, 0.0], [0.000, 0.000, 0.0, 1.000])])
navigate_to_table_2 = NavigateAction([Pose([14.5, 1.5, 0.0], [0, 0, 0.0, 1.000])])
navigate_to_table_2_spoon = NavigateAction([Pose([14.75, 1.4, 0.0], [0, 0, 0.0, 1.000])])


# Returns the orientation of the grasp needed for a top grasp, reference is the robot orientation
def get_grasp_orientation_top():
    orient = lt.bullet_world.robot.pose.orientation
    r_robot = R.from_quat([orient.x, orient.y, orient.z, orient.w])
    r_top = R.from_euler('zx', degrees=True, angles=[-90, 85])
    return (r_robot * r_top).as_quat().tolist()


# Returns the orientation of the grasp needed for a front grasp, reference is the robot orientation
def get_grasp_orientation_front():
    orient = lt.bullet_world.robot.pose.orientation
    r_robot = R.from_quat([orient.x, orient.y, orient.z, orient.w])
    r_front = R.from_euler('z', degrees=True, angles=[-90])
    return (r_robot * r_front).as_quat().tolist()[0]


with real_robot:
    """-------------------------------------"""
    """Basic coffe table demo"""
    """-------------------------------------"""

    # NavigateAction([Pose([11.4, 3.48, 0.0], [0, 0, 1, 1])]).resolve().perform()
    # MoveTorsoAction([0.4]).resolve().perform()
    # MoveJointsMotion(["joint_wrist_yaw"], [-0.14]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0.13, 0.13, 0.13, 0.13]).perform()
    # MoveGripperMotion("close", "arm").perform()
    # MoveTorsoAction([0.6]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # NavigateAction([Pose([13.125, 2.75, 0.0], [0, 0, 1, 0])]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0.1, 0.1, 0.1, 0.1]).perform()
    # MoveTorsoAction([0.4]).resolve().perform()
    # MoveGripperMotion("open", "arm").perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveTorsoAction([0.6]).resolve().perform()
    # NavigateAction([Pose([9.467, 2.117, 0.0], [0, 0, 0, 1])]).resolve().perform()

    # cup = DetectingMotion(ObjectType.JEROEN_CUP).perform()
    # cup_designator = ObjectDesignatorDescription(names=['cup'])

    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveJointsMotion(["joint_wrist_yaw"], [-0.14]).perform()
    # MoveTorsoAction([cup.pose.position.z]).resolve().perform()
    # NavigateAction([Pose([11.7, 3.47, 0.0], [0, 0, 1, 1])]).resolve().perform()
    # cup.pose.position.y -= 0.15
    # cup.pose.position.x -= 0.06
    # cup.pose.position.z += 0.1
    # cup.pose.orientation.x = 0
    # cup.pose.orientation.y = 0
    # cup.pose.orientation.z = 0
    # cup.pose.orientation.w = 1
    # print(cup.pose)
    # MoveTCPMotion(target=cup.pose, arm='arm', allow_gripper_collision=True).perform()
    # giskard.giskard_wrapper.allow_all_collisions()
    # giskard.achieve_cartesian_goal(cup.pose, "link_grasp_center", "base_link")

    # pickup_location = CostmapLocation(target=cup_designator.resolve(), reachable_for=stretch_designator).resolve()
    # print(pickup_location)
    # NavigateAction(target_locations=[pickup_location.pose]).perform()

    """-------------------------------------"""
    """Coffe table to big table demo"""
    """-------------------------------------"""
    # while True:
    # From coffee table to big table
    # MoveTorsoAction([1.08]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # NavigateAction([Pose([11.7, 3.48, 0.0], [0, 0, 1, 1])]).resolve().perform()
    # LookAtAction([Pose([12.8, 3.65, 0.4])]).resolve().perform()
    # yellow_cup = DetectingMotion(ObjectType.YELLOW_CUP).perform()
    # yellow_cup_designator = ObjectDesignatorDescription(names=['yellow_cup'])
    # yellow_cup.pose.position.x -= 0.1
    # yellow_cup.pose.position.y -= 0.14
    # yellow_cup.pose.position.z += 0.12
    # yellow_cup.pose.orientation.x = 0
    # yellow_cup.pose.orientation.y = 0
    # yellow_cup.pose.orientation.z = 0
    # yellow_cup.pose.orientation.w = 1
    #
    # MoveTCPMotion(target=Pose([yellow_cup.pose.position.x - 0.15, yellow_cup.pose.position.y, yellow_cup.pose.position.z],
    #                                                                 get_grasp_orientation_front()), arm='arm',
    #                                                     allow_gripper_collision=True).perform()
    # PickUpAction(yellow_cup_designator, ["arm"], ["front"]).resolve().perform()
    # MoveTorsoAction([1.08]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()

    # NavigateAction([Pose([13.498, 1.803, 0.0])]).resolve().perform()
    # LookAtAction([Pose([13.7, 1.0, 0.8])]).resolve().perform()
    # cup = DetectingMotion(ObjectType.JEROEN_CUP).perform()
    # cup.pose.position.x -= 0.1
    # # cup.pose.position.y -= 0.14
    # cup.pose.position.z += 0.1
    # cup.pose.orientation.x = 0
    # cup.pose.orientation.y = 0
    # cup.pose.orientation.z = -0.7071068
    # cup.pose.orientation.w = 0.7071068
    # PlaceAction(yellow_cup_designator, [Pose([cup.pose.position.x - 0.3, cup.pose.position.y, cup.pose.position.z + 0.1],
    #                                          [0, 0, -0.7071068, 0.7071068])], ["arm"]).resolve().perform()

    # From big table to coffee table
    # MoveTorsoAction([1.08]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # NavigateAction([Pose([13.498, 1.803, 0.0])]).resolve().perform()
    # LookAtAction([Pose([13.7, 1.0, 0.8])]).resolve().perform()
    # yellow_cup = DetectingMotion(ObjectType.YELLOW_CUP).perform()
    # yellow_cup_designator = ObjectDesignatorDescription(names=['yellow_cup'])
    # yellow_cup.pose.position.x -= 0.12
    # yellow_cup.pose.position.y += 0.07
    # yellow_cup.pose.position.z += 0.1
    # yellow_cup.pose.orientation.x = 0
    # yellow_cup.pose.orientation.y = 0
    # yellow_cup.pose.orientation.z = -0.7071068
    # yellow_cup.pose.orientation.w = 0.7071068
    # MoveTCPMotion(
    #     target=Pose([yellow_cup.pose.position.x, yellow_cup.pose.position.y + 0.15, yellow_cup.pose.position.z],
    #                 get_grasp_orientation_front()), arm='arm',
    #     allow_gripper_collision=True).perform()
    # PickUpAction(yellow_cup_designator, ["arm"], ["front"]).resolve().perform()
    # MoveTorsoAction([1.08]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # NavigateAction([Pose([11.7, 3.48, 0.0], [0, 0, 1, 1])]).resolve().perform()
    # LookAtAction([Pose([12.8, 3.65, 0.4])]).resolve().perform()
    # cup = DetectingMotion(ObjectType.JEROEN_CUP).perform()
    # cup.pose.position.x -= 0.1
    # cup.pose.position.y -= 0.14
    # cup.pose.position.z += 0.1
    # cup.pose.orientation.x = 0
    # cup.pose.orientation.y = 0
    # cup.pose.orientation.z = 0
    # cup.pose.orientation.w = 1
    #
    # PlaceAction(yellow_cup_designator,
    #             [Pose([cup.pose.position.x, cup.pose.position.y + 0.2, cup.pose.position.z + 0.09],
    #                   [0, 0, 0, 1])], ["arm"]).resolve().perform()

    """-------------------------------------"""
    """Gripping things"""
    """-------------------------------------"""

    """ yaw: (-1.75, 4.0)
        pitch: (-1.57, 0.56)
        roll: (-3.14, 3.14) """

    # NavigateAction([Pose([11.7, 3.48, 0.0], [0, 0, 1, 1])]).resolve().perform()

    """BOWL"""
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveTorsoAction([0.7]).resolve().perform()
    # MoveJointsMotion(["joint_wrist_yaw"], [-0.14]).perform()
    # MoveJointsMotion(["joint_wrist_pitch"], [-1.2]).perform()
    # MoveJointsMotion(["joint_wrist_roll"], [-1.5]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"],
    #                  [0.12, 0.12, 0.12, 0.12]).perform()
    # MoveTorsoAction([0.58]).resolve().perform()
    # MoveGripperMotion("close", "arm").perform()
    # MoveTorsoAction([0.7]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()

    """CUTLERY"""
    # MoveGripperMotion("open", "arm").perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveTorsoAction([0.7]).resolve().perform()
    # MoveJointsMotion(["joint_wrist_yaw"], [-0.14]).perform()
    # MoveJointsMotion(["joint_wrist_pitch"], [-1.0]).perform()
    # MoveJointsMotion(["joint_wrist_roll"], [0.0]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"],
    #                  [0.12, 0.12, 0.12, 0.12]).perform()
    # MoveTorsoAction([0.54]).resolve().perform()
    # MoveGripperMotion("close", "arm").perform()
    # MoveTorsoAction([0.7]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()

    """PLATE"""
    # MoveGripperMotion("open", "arm").perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveTorsoAction([0.7]).resolve().perform()
    # MoveJointsMotion(["joint_wrist_yaw"], [-0.14]).perform()
    # MoveJointsMotion(["joint_wrist_pitch"], [-0.3]).perform()
    # MoveJointsMotion(["joint_wrist_roll"], [-1.5]).perform()
    # MoveTorsoAction([0.45]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"],
    #                  [0.12, 0.12, 0.12, 0.12]).perform()
    #
    # MoveGripperMotion("close", "arm").perform()
    # MoveTorsoAction([0.7]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()

    """-------------------------------------"""
    """Bowl detection"""
    """-------------------------------------"""

    # MoveTorsoAction([0.9]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # # MoveJointsMotion(["joint_wrist_yaw"], [-0.14]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # NavigateAction([Pose([11.825, 3.092, 0.0], [0, 0, 1, 1])]).resolve().perform()
    # LookAtAction([Pose([12.4, 3.75, 0.4])]).resolve().perform()
    # bowl = DetectingMotion(ObjectType.BOWL).perform()
    # bowl_designator = ObjectDesignatorDescription(names=['bowl']).resolve()
    #
    # bowl.pose.position.y -= 0.18
    # bowl.pose.position.x -= 0.04
    # bowl.pose.position.z += 0.12
    # bowl.pose.orientation.x = 0
    # bowl.pose.orientation.y = 0
    # bowl.pose.orientation.z = 0
    # bowl.pose.orientation.w = 1
    # MoveGripperMotion("open", "arm").perform()
    # PickUpAction(bowl_designator, ["arm"], ["top"]).resolve().perform()
    # MoveTorsoAction([0.9]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()

    """-------------------------------------"""
    """Spoon detection"""
    """-------------------------------------"""

    # MoveTorsoAction([1.0]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    # MoveGripperMotion("open", "arm").perform()
    # NavigateAction([Pose([14.4, 1.62, 0.0], [0, 0, -0.7, 0.7])]).resolve().perform()
    # LookAtAction([Pose([14.53, 1.3, 0.83])]).resolve().perform()
    #
    # spoon = DetectingMotion(ObjectType.SPOON).perform()
    # spoon_designator = ObjectDesignatorDescription(names=['spoon']).resolve()
    #
    # spoon.pose.position.y += 0.16
    # spoon.pose.position.x += 0.04
    # spoon.pose.position.z += 0.11
    # spoon.pose.orientation.x = 0
    # spoon.pose.orientation.y = 0
    # spoon.pose.orientation.z = -0.8
    # spoon.pose.orientation.w = 1
    # MoveGripperMotion("open", "arm").perform()
    # NavigateAction([Pose([14.53, 1.82, 0.0], [0, 0, 0, 1.0])]).resolve().perform()
    # PickUpAction(spoon_designator, ["arm"], ["top"]).resolve().perform()
    # MoveTorsoAction([1.0]).resolve().perform()
    # MoveJointsMotion(["joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_arm_l0"], [0., 0., 0., 0.]).perform()
    """-------------------------------------"""
    """Cleaning table demo"""
    """-------------------------------------"""

    # open_gripper.perform()
    # move_torso_up.resolve().perform()
    # retrieve_arm_for_detecting.perform()
    # before_pickup_plan.perform()
    # looking.resolve().perform()
    #
    # bowl = DetectingMotion(ObjectType.BOWL).perform()
    # bowl_designator = ObjectDesignatorDescription(names=['bowl']).resolve()
    # # bowl.pose.position.y += 0.18
    # bowl.pose.position.x -= 0.165
    # bowl.pose.position.z += 0.12
    # bowl.pose.orientation.x = 0
    # bowl.pose.orientation.y = 0
    # bowl.pose.orientation.z = -0.72
    # bowl.pose.orientation.w = 0.72
    #
    # MoveTCPMotion(target=Pose([bowl.pose.position.x + 0.03, bowl.pose.position.y, bowl.pose.position.z + 0.07],
    #                           get_grasp_orientation_top()), arm='arm',
    #               allow_gripper_collision=True).perform()
    # PickUpAction(bowl_designator, ["arm"], ["top"]).resolve().perform()
    # move_torso_up.resolve().perform()
    # retrieve_arm_fully.perform()
    # navigate_to_small_table.resolve().perform()
    #
    # LookAtAction([Pose([12.8, 3.8, 0.4])]).resolve().perform()
    #
    # yellow_cup = DetectingMotion(ObjectType.YELLOW_CUP).perform()
    # yellow_cup.pose.position.y -= 0.11
    # yellow_cup.pose.position.x += 0.015
    # yellow_cup.pose.position.z += 0.12
    # yellow_cup.pose.orientation.x = 0
    # yellow_cup.pose.orientation.y = 0
    # yellow_cup.pose.orientation.z = 0
    # yellow_cup.pose.orientation.w = 1
    #
    # PlaceAction(bowl_designator, [Pose([yellow_cup.pose.position.x, yellow_cup.pose.position.y - 0.32,
    #                                     yellow_cup.pose.position.z + 0.07])], ["arm"]).resolve().perform()
    # move_torso_up.resolve().perform()
    # retrieve_arm_for_detecting.perform()
    # before_pickup_plan.perform()
    # looking.resolve().perform()
    #
    # cup = DetectingMotion(ObjectType.JEROEN_CUP).perform()
    # cup_designator = ObjectDesignatorDescription(names=['cup'])
    # cup.pose.position.y -= 0.05
    # cup.pose.position.x -= 0.155
    # cup.pose.position.z += 0.12
    # cup.pose.orientation.x = 0
    # cup.pose.orientation.y = 0
    # cup.pose.orientation.z = -0.72
    # cup.pose.orientation.w = 0.72
    #
    # MoveTCPMotion(target=Pose([cup.pose.position.x + 0.04, cup.pose.position.y + 0.09, cup.pose.position.z],
    #                           get_grasp_orientation_front()), arm='arm',
    #               allow_gripper_collision=True).perform()
    # PickUpAction(cup_designator, ["arm"], ["front"]).resolve().perform()
    # move_torso_up.resolve().perform()
    # retrieve_arm_fully.perform()
    # navigate_to_small_table.resolve().perform()
    # PlaceAction(cup_designator, [Pose([yellow_cup.pose.position.x, yellow_cup.pose.position.y - 0.55,
    #                                    yellow_cup.pose.position.z + 0.095])], ["arm"]).resolve().perform()

    # move_torso_up.resolve().perform()
    # retrieve_arm_for_detecting.perform()
    # before_pickup_plan_spoon.perform()
    #
    # spoon = DetectingMotion(ObjectType.SPOON).perform()
    # spoon_designator = ObjectDesignatorDescription(names=['spoon']).resolve()
    # spoon.pose.position.y += 0.04
    # spoon.pose.position.x -= 0.11
    # spoon.pose.position.z += 0.105
    # spoon.pose.orientation.x = 0
    # spoon.pose.orientation.y = 0
    # spoon.pose.orientation.z = -0.72
    # spoon.pose.orientation.w = 0.72
    #
    # MoveTCPMotion(target=Pose([spoon.pose.position.x, spoon.pose.position.y, spoon.pose.position.z + 0.05],
    #                           get_grasp_orientation_top()), arm='arm',
    #               allow_gripper_collision=True).perform()
    # PickUpAction(spoon_designator, ["arm"], ["top"]).resolve().perform()
    # move_torso_up.resolve().perform()
    # retrieve_arm_fully.perform()
    # navigate_to_small_table.resolve().perform()
    #
    # LookAtAction([Pose([12.8, 3.8, 0.4])]).resolve().perform()
    #
    # yellow_cup = DetectingMotion(ObjectType.YELLOW_CUP).perform()
    # yellow_cup.pose.position.y -= 0.11
    # yellow_cup.pose.position.x += 0.015
    # yellow_cup.pose.position.z += 0.12
    # yellow_cup.pose.orientation.x = 0
    # yellow_cup.pose.orientation.y = 0
    # yellow_cup.pose.orientation.z = 0
    # yellow_cup.pose.orientation.w = 1
    #
    # PlaceAction(spoon_designator, [Pose([yellow_cup.pose.position.x, yellow_cup.pose.position.y - 0.32,
    #                                     yellow_cup.pose.position.z + 0.12])], ["arm"]).resolve().perform()
    #
    # move_torso_up.resolve().perform()
    # retrieve_arm_fully.perform()

    """-------------------------------------"""
    """Cleaning and setting table loop demo"""
    """-------------------------------------"""


    # Moves the robot arm to a pre-pickup position (in front or above object)
    def move_tcp(obj_type, obj, task):
        if obj_type == 'yellow_cup':
            MoveTCPMotion(target=Pose([obj.pose.position.x + 0.04, obj.pose.position.y + 0.15, obj.pose.position.z],
                                      get_grasp_orientation_front()), arm='arm',
                          allow_gripper_collision=True).perform()
        elif obj_type == 'bowl':
            MoveTCPMotion(
                target=Pose([obj.pose.position.x + 0.05, obj.pose.position.y + 0.05, obj.pose.position.z + 0.08],
                            get_grasp_orientation_top()), arm='arm',
                allow_gripper_collision=True).perform()
        elif obj_type == 'spoon':
            MoveTCPMotion(target=Pose([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z + 0.1],
                                      get_grasp_orientation_top()), arm='arm',
                          allow_gripper_collision=True).perform()

    def before_pickup(obj_type, task):
        """
        Function for preparing to pick up object
        Detection and error handlig
        """
        # For the spoon we need a different navigation, the robot needs to be closer to perceive it
        if obj_type == 'spoon':
            # Before pickup plan for cleaning table with spoon
            if task == "clean":
                plan = open_gripper + park_arms + navigate_to_table_1_spoon + looking_spoon

            # Before pickup plan for setting table with spoon
            else:
                plan = open_gripper + park_arms + navigate_to_table_2_spoon + looking_spoon_table_2
            plan.perform()

        # Pickup plan for bowl and cup
        else:
            # Before pickup plan for cleaning table with objects other than spoon
            if task == "clean":
                plan = open_gripper + park_arms + navigate_to_table_1 + looking

            # Before pickup plan for setting table with objects other than spoon
            else:
                plan = open_gripper + park_arms + navigate_to_table_2 + looking_cup
            plan.perform()

        """
        Perception loop with error handling

        1. The robot tries to detect object
        2. If the robot sees the object but the depth is not enough (usually with spoon), throws IndexError. In this 
        case, the robot drives forward a bit and tries to detect again, maximum 5 times. If it reaches 5 times, the
        robot drives back to the original position and tries it all again
        3. Other errors mean the object is not in the view field or there is a robkudo bug, the robot looks again
        and detects
        """

        cnt = 0
        while True:
            try:
                obj = detect_object(obj_type, task)
                obj_designator = ObjectDesignatorDescription(names=[obj_type])

            # Error handling if the robot sees the object but the depth is not enough
            # Goes forward a bit and perceives again
            except (IndexError):
                if cnt < 5:
                    NavigateAction(
                        [Pose([lt.bullet_world.robot.pose.position.x + 0.1, lt.bullet_world.robot.pose.position.y,
                               lt.bullet_world.robot.pose.position.z],
                              [0.000, 0.000, 0.0, 1.000])]).resolve().perform()
                    if task == "clean":
                        looking_spoon.resolve().perform()
                    else:
                        looking_spoon_table_2.resolve().perform()
                    cnt += 1

                else:
                    if task == "clean":
                        navigate_to_table_1_spoon.resolve().perform()
                        looking_spoon.resolve().perform()
                    else:
                        navigate_to_table_2_spoon.resolve().perform()
                        looking_spoon_table_2.resolve().perform()
                    cnt = 0
                continue
            except (NameError, TypeError, AttributeError):
                # With other errors, we just perceive again
                if task == "clean":
                    if obj_type == "spoon":
                        looking_spoon.resolve().perform()
                    else:
                        looking.resolve().perform()
                else:
                    if obj_type == "spoon":
                        looking_spoon_table_2.resolve().perform()
                    else:
                        looking_cup.resolve().perform()
                continue
            return obj, obj_designator


    def try_pickup(obj_type, task):
        """
        Function to pick up object, error handling if object wasn't grasped

        :param obj_type: type of object as string ("spoon", "yellow_cup", "bowl")
        :param task: task the robot should perform as string ("clean", "set")
        """

        obj, obj_designator = before_pickup(obj_type, task)

        # Pickup plan loop with error handling
        while True:
            # Move gripper to pre-position
            move_tcp(obj_type, obj, task)

            # For the cup we need front grasp
            if obj_type == 'yellow_cup':
                PickUpAction(obj_designator, ["arm"], ["front"]).resolve().perform()

            # For the other objects we use top grasp
            else:
                PickUpAction(obj_designator, ["arm"], ["top"]).resolve().perform()

            retrieve_arm_fully.perform()

            # The robot turns back to the table and looks at where the object should be for error handling
            if obj_type == 'spoon':
                if task == "clean":
                    navigate_to_table_1_spoon.resolve().perform()
                    NavigateAction([Pose([lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y,
                                          lt.bullet_world.robot.pose.position.z],
                                         [0.000, 0.000, -1.000, 1.000])]).resolve().perform()
                    looking_spoon.resolve().perform()
                else:
                    navigate_to_table_2_spoon.resolve().perform()
                    NavigateAction([Pose([lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y,
                                          lt.bullet_world.robot.pose.position.z],
                                         [0.000, 0.000, -1.000, 1.000])]).resolve().perform()
                    looking_spoon_table_2.resolve().perform()

            # Looking poses for objects other than spoon
            else:

                if task == "clean":
                    NavigateAction([Pose([lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y,
                                          lt.bullet_world.robot.pose.position.z],
                                         [0.000, 0.000, -1.000, 1.000])]).resolve().perform()
                    looking.resolve().perform()
                else:
                    NavigateAction([Pose([lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y,
                                          lt.bullet_world.robot.pose.position.z],
                                         [0.000, 0.000, -1.000, 1.000])]).resolve().perform()
                    looking_cup.resolve().perform()

            # Checking if pickup was successful with trying to detect again
            try:
                robot_position = (lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y)

                # If detect_object throws error the pickup was successful, we skip the rest
                if obj_type == 'spoon':
                    while True:
                        try:
                            obj_try = detect_object(obj_type, task)

                        # If spoon is there but depth is not enough, the spoon is still on the table,
                        # so the robot tries the pickup again
                        except (IndexError, AttributeError):
                            if task == "clean":
                                NavigateAction(
                                    [Pose([lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y - 0.08,
                                          lt.bullet_world.robot.pose.position.z], [0.000, 0.000, -1.011, 1.000])]).resolve().perform()
                                looking_spoon.resolve().perform()
                            else:
                                NavigateAction(
                                    [Pose([lt.bullet_world.robot.pose.position.x, lt.bullet_world.robot.pose.position.y - 0.08,
                                          lt.bullet_world.robot.pose.position.z], [0.000, 0.000, -1.011, 1.000])]).resolve().perform()
                                looking_spoon_table_2.resolve().perform()
                            continue
                        break
                else:
                    while True:
                        try:
                            obj_try = detect_object(obj_type, task)
                        except (AttributeError):
                            if task == "clean":
                                looking.resolve().perform()
                            else:
                                looking_cup.resolve().perform()
                            continue
                        break

                # If the detected object is outside the tolerance, we exit the loop, the pickup was successful,
                # the detected object was not our object
                if check_false_detection(obj_try, robot_position):
                    break

                # If the loop continues the pickup was unsuccessful as detect_object didn't throw exeption
                # We detach the object from the robot, and do the pickup plan again, the same way as before
                BulletWorld.robot.detach(world.get_objects_by_name(obj_type)[0])
                before_pickup(obj_type, task)
            except Exception as e:
                print("exception in try_pickup")
                traceback.print_exc()
                print(e)
                break
        return obj_designator


    # Funcion for placing an object
    def place(obj_desig, obj_type, task):
        # We need different plan for cleaning and setting the table
        # Plan for cleaning table
        if task == "clean":
            navigate_to_table_2.resolve().perform()
            looking_cup.resolve().perform()

            # Different offsets for placing objects relative to each other
            # Place spoon relative to bowl
            if obj_type == 'spoon':
                # Error handling for not detecting bowl
                while True:
                    try:
                        bowl = detect_object('bowl', "set")
                    except (NameError, TypeError, AttributeError):
                        continue
                    break
                x_offset = -0.15
                y_offset = 0.06
                z_offset = 0.03
                PlaceAction(obj_desig, [Pose([bowl.pose.position.x + x_offset, bowl.pose.position.y + y_offset,
                                              bowl.pose.position.z + z_offset], [0, 0, -0.7071068, 0.7071068])],
                            ["arm"]).resolve().perform()
                return

            # Place bowl relative to cup with offsets
            elif obj_type == 'bowl':
                x_offset = 0.0#4
                y_offset = 0.3
                z_offset = 0.08

            # Place yellow_cup relative to cup with offsets
            else:
                x_offset = -0.25
                y_offset = 0.11
                z_offset = 0.09

            # Error handling for not detecting cup
            while True:
                try:
                    cup = detect_object('cup', "set")
                except (NameError, TypeError, AttributeError):
                    continue
                break
            PlaceAction(obj_desig, [Pose([cup.pose.position.x + x_offset, cup.pose.position.y + y_offset,
                                          cup.pose.position.z + z_offset], [0, 0, -0.7071068, 0.7071068])],
                        ["arm"]).resolve().perform()

        # Plan for setting the table
        else:
            navigate_to_table_1.resolve().perform()
            looking.resolve().perform()

            # Place spoon relative to bowl (with perception error handling)
            if obj_type == 'spoon':
                while True:
                    try:
                        bowl = detect_object('bowl', "clean")
                    except (NameError, TypeError, AttributeError, IndexError):
                        continue
                    break
                x_offset = 0.23
                y_offset = 0.12
                z_offset = 0.05
                PlaceAction(obj_desig, [Pose([bowl.pose.position.x + x_offset, bowl.pose.position.y + y_offset,
                                              bowl.pose.position.z + z_offset], [0, 0, -0.7071068, 0.7071068])],
                            ["arm"]).resolve().perform()

            # Place bowl relative to yellow_cup (with perception error handling)
            elif obj_type == 'bowl':
                while True:
                    try:
                        cup = detect_object('cup', "clean")
                    except (NameError, TypeError, AttributeError):
                        continue
                    break
                x_offset = 0.1
                y_offset = 0.15
                z_offset = 0.08
                PlaceAction(obj_desig, [Pose([cup.pose.position.x + x_offset, cup.pose.position.y + y_offset,
                                              cup.pose.position.z + z_offset], [0, 0, -0.7071068, 0.7071068])],
                            ["arm"]).resolve().perform()

            # Place yellow_cup relative to blue cup (with perception error handling)
            else:
                while True:
                    try:
                        cup = detect_object('cup', "clean")
                    except (NameError, TypeError, AttributeError):
                        continue
                    break
                x_offset = 0.2
                y_offset = 0.05
                z_offset = 0.1
                PlaceAction(obj_desig, [Pose([cup.pose.position.x + x_offset, cup.pose.position.y + y_offset,
                                              cup.pose.position.z + z_offset], [0, 0, -0.7071068, 0.7071068])],
                            ["arm"]).resolve().perform()


    # Offset of objects when detecting, change if needed in new environment
    # When to change what?
    # Offsets can change after restarting the robot, change them according to need (orientations should be always good)
    # Different offsets for detecting by different tasks (by "clean" the robot detects at table1, by "set"
    # the robot detects at table2)
    def detect_object(obj_type, task):
        obj = None
        if obj_type == "cup":
            if task == "clean":
                obj = DetectingMotion(ObjectType.JEROEN_CUP).perform()
                obj.pose.position.x -= 0.16
                obj.pose.position.y += 0.08
                obj.pose.position.z += 0.12
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068
            else:
                obj = DetectingMotion(ObjectType.JEROEN_CUP).perform()
                obj.pose.position.x -= 0.16
                obj.pose.position.y += 0.0
                obj.pose.position.z += 0.12
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068



        elif obj_type == "bowl":
            obj = DetectingMotion(ObjectType.BOWL).perform()
            if task == "clean":
                obj.pose.position.x -= 0.18
                obj.pose.position.y += 0.04
                obj.pose.position.z += 0.12
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068
            else:
                obj.pose.position.x -= 0.14
                obj.pose.position.y += 0.09
                obj.pose.position.z += 0.12
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068


        elif obj_type == "spoon":
            obj = DetectingMotion(ObjectType.SPOON).perform()
            if task == "clean":
                obj.pose.position.x -= 0.1
                obj.pose.position.y += 0.025
                obj.pose.position.z += 0.105
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068
            else:
                obj.pose.position.x -= 0.1
                obj.pose.position.y += 0.07
                obj.pose.position.z += 0.105
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068



        elif obj_type == "yellow_cup":
            obj = DetectingMotion(ObjectType.YELLOW_CUP).perform()
            if task == "clean":
                obj.pose.position.x -= 0.11
                obj.pose.position.y -= 0.01
                obj.pose.position.z += 0.12
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068
            else:
                obj.pose.position.x -= 0.11
                obj.pose.position.y += 0.03
                obj.pose.position.z += 0.12
                obj.pose.orientation.x = 0
                obj.pose.orientation.y = 0
                obj.pose.orientation.z = -0.7071068
                obj.pose.orientation.w = 0.7071068

        return obj


    # Function for picking up and placing an object
    def pick_and_place(obj_type, task):
        obj_desig = try_pickup(obj_type, task)
        place(obj_desig, obj_type, task)


    # Check if the detected object is too close or too far from the robot
    def check_false_detection(obj, robot_position):
        obj_position = (obj.pose.position.x, obj.pose.position.y)
        print(distance.euclidean(robot_position, obj_position))
        if distance.euclidean(robot_position, obj_position) > 1.0 or distance.euclidean(robot_position,
                                                                                        obj_position) < 0.46:
            return True
        return False


    faulthandler.enable()

    # Main loop function
    # If an error occurred, comment out the tasks that are not needed and start the demo again
    while True:
        pick_and_place('bowl', "clean")
        pick_and_place('yellow_cup', "clean")
        pick_and_place('spoon', "clean")
        # break
        # pick_and_place('yellow_cup', "set")
        # pick_and_place('bowl', "set")
        # pick_and_place('spoon', "set")

        # park_arms.resolve().perform()
        # open_gripper.resolve().perform()
        # close_gripper.resolve().perform()
        #### obj_desig = try_pickup('yellow_cup', "clean")
        #### # park_arms.resolve().perform()
        # NavigateAction([Pose([12.5, 2.8, 0.0], [0, 0, 0.7071068, 0.7071068])]).resolve().perform()
        # NavigateAction([Pose([12.5, 2.8, 0.0], [0, 0, 1, 0])]).resolve().perform()
        # LookAtAction([Pose([12.4, 4.9, 1])]).resolve().perform()
        # MoveTCPMotion(target=Pose([12.8, 3.47, 0.8],
        #                           [0, 0, 0.7071068, 0.7071068]), arm='arm',
        #               allow_gripper_collision=True).perform()
        # open_gripper.resolve().perform()
        # park_arms.resolve().perform()
        # obj = detect_object("yellow_cup", "clean")
        # obj_designator = ObjectDesignatorDescription(names=["yellow_cup"])
        # close_gripper.resolve().perform()
        break