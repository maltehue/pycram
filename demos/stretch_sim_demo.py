
from pycram.worlds.bullet_world import *
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.designators.motion_designator import *
from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.designators.location_designator import CostmapLocation
from pycram.designators.object_designator import BelieveObject

world = BulletWorld(WorldMode.GUI)
stretch = Object("stretch", ObjectType.ROBOT, "stretch.urdf")
stretch_designator = ObjectDesignatorDescription(names=['stretch']).resolve()
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1, 0, 0.5]))
milk_desig = BelieveObject(names=['milk'])

with simulated_robot:
    MoveGripperMotion("open", "arm").perform()
    MoveTorsoAction([0.4]).resolve().perform()
    pickup_location = CostmapLocation(target=milk_desig.resolve(), reachable_for=stretch_designator).resolve()
    NavigateAction(target_locations=[pickup_location.pose]).resolve().perform()
