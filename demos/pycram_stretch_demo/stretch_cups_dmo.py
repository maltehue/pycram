import time
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode,Arms
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import real_robot, with_real_robot
from pycram.world_concepts.world_object import Object
from pycram.ros_utils.robot_state_updater import WorldStateUpdater
from pycrap.ontologies import *
from pycram.datastructures.dataclasses import Color




world = BulletWorld(mode=WorldMode.DIRECT)


stretch= Object('stretch_description',Robot,'stretch_description.urdf')
stretch_designator = ObjectDesignatorDescription(names=['stretch_description']).resolve()

apartment=Object('apartment',Apartment,'apartment.urdf')



robot_desig=BelieveObject(names=["stretch_description"])


red_cup=Object("red_cup",Cup,"jeroen_cup.stl",pose=PoseStamped.from_list([17,0.5,1],[0,0,0,1]))
red_cup_desig=BelieveObject(names=["red_cup"])

blue_cup=Object("blue_cup",Cup,"jeroen_cup.stl",pose=PoseStamped.from_list([16.7,0.5,1],[0,0,0,1]))
blue_cup_desig=BelieveObject(names=["blue_cup"])


WorldStateUpdater(tf_topic="/tf", joint_state_topic="/joint_states")



#This two colors are needed for this demo
blue=Color(0, 0, 1, 1)
red=Color(1, 0, 0, 1)




@with_real_robot
def detect(cup,cup_desig,color:Color,offset_x):
    """
    Detects the given cup using the robot's perception system.
    Updates the cup's color, applies an offset to the detected pose,
    and updates the object designator's pose.

    Args:
        cup: The cup object instance.
        cup_desig: The object designator representing the cup.
        color (Color): The color to assign to the cup.
        offset_x (float): X-axis offset to apply to the detected pose.

    Returns:
        cup_desig: The updated object designator with the new pose.

    Exceptions:
        Prints any exceptions encountered during detection.
    """

    print(f"[DETECTING] Detecting cup {cup.name}...")
    try:
        cup.color = color
        detect_cup=DetectActionDescription(technique=DetectionTechnique.ALL, object_designator=cup_desig).resolve().perform()
        offset_pose=detect_cup.pose
        offset_pose.position.x-=offset_x
        cup_desig.resolve().pose=offset_pose

    except Exception as e:
        print(e)

    return cup_desig


def wait_for_c_key():
    """
    Waits for the user to press 'c' and Enter to continue.
    Used for manual step-wise control during the demo.
    """
    while True:
        key = input("Press 'c' and Enter to continue: ")
        if key.strip().lower() == 'c':
            break


@with_real_robot
def navigate_to(location, hand_empty=False):
    """
        Navigates the robot to the specified location, adjusting the arm posture if necessary.

        Args:
            location: The target location to navigate to.
            hand_empty (bool): If True, moves the arm to an 'empty hand' posture before navigation.

        Exceptions:
            Prints any exceptions encountered during navigation.
        """

    wait_for_c_key()
    try:
        MoveJointsMotion(["joint_lift"], [1]).perform()

        if hand_empty:
            MoveJointsMotion(["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3", "joint_wrist_yaw"], [0., 0., 0., 0., 3]).perform()
        else:
            MoveJointsMotion(["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"], [0., 0., 0., 0.]).perform()
            MoveJointsMotion(['joint_wrist_yaw'], [0]).perform()
        print(f"[NAVIGATE] Navigating to {location}...")

        NavigateActionDescription([location]).resolve().perform()
    except Exception as e:
        print(e)


@with_real_robot
def pick_up_object(cup_desig,grsp):
    """
    Picks up the specified object (e.g., a cup) using the given grasp side.

    Args:
        cup_desig: The object designator for the cup/object.
        grsp (str): Grasp type, 'left' or 'right' in this demo.

    Exceptions:
        Prints any exceptions encountered during the pick-up action.
    """

    print(f"[Picking] Picking Cup ...")

    #Note:if we are grasping from table use grasp.right
    try:
        if grsp == "left":
            grasp = GraspDescription(Grasp.LEFT, None, False)
        elif grsp == "right":
            grasp = GraspDescription(Grasp.RIGHT, None, False)
        PickUpActionDescription(object_designator=cup_desig,
                                arm=Arms.RIGHT,
                                grasp_description=grasp).perform()

    except Exception as e:
        print(e)

    MoveJointsMotion(["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"], [0., 0., 0., 0.]).perform()
    MoveJointsMotion(['joint_wrist_yaw'], [0]).perform()


@with_real_robot
def place_object(location,obj_des):
    """
    Places the specified object at the given location using the robot's arm.

    Args:
        location: The target pose/location to place the object.
        obj_des: The object designator to be placed.

    Exceptions:
        Prints any exceptions encountered during the place action.
    """

    print(f"[PLACE] Placing object at {location}...")
    try:
        MoveTCPMotion(target=location, arm=Arms.RIGHT).perform()
        SetGripperActionDescription(gripper=Arms.RIGHT, motion=[GripperState.OPEN]).resolve().perform()
        stretch.detach(obj_des)

    except Exception as e:
        print(e)

    ParkArmsActionDescription([Arms.RIGHT]).resolve().perform()
    SetGripperActionDescription(gripper=Arms.RIGHT, motion=[GripperState.CLOSE]).resolve().perform()


@with_real_robot
def placing_back(obj_des):
    """
    Places the specified object (e.g., a cup) back to the shelf by extending the Arm joints to reach the shelf.

    Args:
        obj_des: The object designator for the object to be placed back.

    Exceptions:
        Prints any exceptions encountered and parks the arm/closes the gripper on error.
    """

    print(f"[PLACE] Placing Cup Back...")
    try:
        MoveJointsMotion(["joint_arm_l0", "joint_arm_l1","joint_arm_l2"], [0.1, 0.07,0.07]).perform()
        MoveJointsMotion(["joint_lift"], [0.975]).perform()
        SetGripperActionDescription(gripper=Arms.RIGHT, motion=[GripperState.OPEN]).resolve().perform()
        MoveJointsMotion(["joint_lift"], [1.09]).perform()
        stretch.detach(obj_des)


    except Exception as e:
        print(e)

    ParkArmsActionDescription([Arms.RIGHT]).resolve().perform()
    SetGripperActionDescription(gripper=Arms.RIGHT, motion=[GripperState.CLOSE]).resolve().perform()


@with_real_robot
def look_at_pose(position):
    """
    Orients the robot's camera to look at the specified position.

    Args:
        position: The pose or coordinates for the robot to look at.

    Exceptions:
        Prints any exceptions encountered during the action.
    """

    print(f"[LOCKING] Locking at {position}...")
    try:

     LookAtActionDescription([position]).resolve().perform()


    except Exception as e:
        print(e)

    MoveJointsMotion(["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"], [0., 0., 0., 0.]).perform()






with real_robot:
    for i in range(4):
        print("===================================================FIRST PART==============================================================================")


        print("======================STARTING DEMO WITH BLUE CUP=======================")

        ParkArmsActionDescription([Arms.RIGHT]).resolve().perform()
        navigate_to(PoseStamped.from_list([16.75, 1, 0], [0, 0, 0, 1]))
        look_at_pose(PoseStamped.from_list([17, 0.4, 1.2]))
        det_blue_cup = detect(blue_cup, blue_cup_desig, blue, 0.11)
        pick_up_object(det_blue_cup, "left")
        navigate_to(PoseStamped.from_list([17.5,2.6,0],[0,0,1,0]))
        look_at_pose(PoseStamped.from_list([17.2, 3.3, 0.5]))
        place_object(PoseStamped.from_list([17.2, 3.2, 0.53],[0,0,0.7071,0.7071]),det_blue_cup)
        blue_cup.color=Color(0, 0, 0, 1)

        print("======================FINISHED WITH BLUE CUP============================")
        print("=========================STARTING RED CUP================================")

        navigate_to(PoseStamped.from_list([16.9, 1, 0], [0, 0, 0, 1]))
        look_at_pose(PoseStamped.from_list([17.2, 0.4, 1.1]))
        det_red_cup = detect(red_cup, red_cup_desig, red,0.125)
        pick_up_object(det_red_cup,"left")
        navigate_to(PoseStamped.from_list([17.6, 2.7, 0], [0, 0, 1, 0]))
        look_at_pose(PoseStamped.from_list([17.5, 3.3, 0.5]))
        place_object(PoseStamped.from_list([17.4, 3.2, 0.52], [0, 0, 0.7071, 0.7071]),det_red_cup)
        red_cup.color = Color(0, 0, 0, 1)

        print("=========================FINISHED WITH BLUE CUP===========================")

        print("======================================FINISHED WITH FIRST PART=================================================")


        print("=========================================  SECOND PART ================================================")
        print("=========================BLUE CUP PART II ==================================")

        navigate_to(PoseStamped.from_list([17.55, 2.56, 0], [0, 0, 1, 0]))
        look_at_pose(PoseStamped.from_list([17.1, 3.3, 0.4]))

        for i in range(3):
            det_blue_cup2 = detect(blue_cup, blue_cup_desig, blue, -0.035)
            zb=det_blue_cup2.resolve().pose.position.z
            if  zb> 0.6:
                print("z value doesnt make sense: ", zb)
                print("we should detect again")
                blue_cup.color = Color(0, 0, 0, 1)
                time.sleep(5)
            else:
                print("z value make sense: ",zb)
                det_blue_cup2.resolve().pose.position.z=zb
                print(det_blue_cup2.resolve().pose)
                pick_up_object(det_blue_cup2,"right")
                print("picked")
                break


        navigate_to(PoseStamped.from_list([17.1, 1, 0], [0, 0, 0, 1]))
        placing_back(det_blue_cup2)
        blue_cup.color = Color(0, 0, 0, 1)


        print("=========================FINISHED WITH BLUE CUP==================================")

        print("=========================RED CUP PART II ==================================")

        navigate_to(PoseStamped.from_list([17.2, 2, 0], [0, 0, 1, 1]))
        navigate_to(PoseStamped.from_list([17.7, 2.56, 0], [0, 0, 1, 0]))
        look_at_pose(PoseStamped.from_list([17.4, 3.3, 0.5]))


        for i in range(3):
            det_red_cup2 = detect(red_cup, red_cup_desig, red, -0.035)
            zr=det_red_cup2.resolve().pose.position.z
            if  zr> 0.6:
                print("z value doesnt make sense: ", zr)
                print("we should detect again")
                red_cup.color = Color(0, 0, 0, 1)
                time.sleep(5)
            else:
                print("z value make sense: ",zr)
                det_red_cup2.resolve().pose.position.z=zr
                pick_up_object(det_red_cup2,"right")
                print("picked")
                break


        navigate_to(PoseStamped.from_list([17.23, 1, 0], [0, 0, 0, 1]))
        placing_back(det_red_cup2)
        red_cup.color = Color(0, 0, 0, 1)

        print("=========================FINISHED WITH RED CUP==================================")

        print(f"********************** loop {i+1} finished ********************************")

