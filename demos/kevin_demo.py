from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap import Robot, Apartment, Milk, Cereal, Spoon, Bowl
import numpy as np
from pycram.ros_utils.robot_state_updater import WorldStateUpdater
from datetime import timedelta


np.random.seed(420)
extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.GUI)

robot = Object("kevin", Robot, f"kevin{extension}", pose=Pose([1, 2, 0]))
# WorldStateUpdater(tf_topic="/tf", joint_state_topic="/joint_states", update_rate=timedelta(seconds=2),
#                   world=world)
apartment = Object("apartment", Apartment, f"apartment{extension}")

milk = Object("milk", Milk, "milk.stl", pose=Pose([2.5, 2, 1.02]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=Pose([2.5, 2.3, 1.05]), color=Color(0, 1, 0, 1))
spoon = Object("spoon", Spoon, "spoon.stl", pose=Pose([2.4, 2.24, 0.85]),
               color=Color(0, 0, 1, 1))
bowl = Object("bowl", Bowl, "bowl.stl", pose=Pose([2.5, 2.2, 1.02]),
              color=Color(1, 1, 0, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["kevin"])
apartment_desig = BelieveObject(names=["apartment"])

@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.4, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(technique=DetectionTechnique.TYPES, object_designator_description=BelieveObject(types=[obj_type])).resolve().perform()
    return object_desig[0]

with simulated_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()

    milk_desig = move_and_detect(Milk)
    #
    MoveTCPMotion(milk_desig.pose, Arms.LEFT).perform()
    # TransportAction(milk_desig, [Pose([4.8, 3.55, 0.8])], [Arms.LEFT]).resolve().perform()

    # from pycram.designators.specialized_designators.location.giskard_location import GiskardLocation
    #
    # robot_desig = BelieveObject(names=["kevin"]).resolve()
    #
    # loc = GiskardLocation(target=Pose([2.5, 2, 1.02]), reachable_for=robot_desig).resolve()
    # print(loc.pose)
