from ...external_interfaces.giskard import achieve_cartesian_goal
from ...designators.location_designator import CostmapLocation
from pycram.world import UseProspectionWorld, World
from pycram.datastructures.pose import Pose
from ...robot_descriptions import robot_description
from pycram.pose_generator_and_validator import reachability_validator
from typing_extensions import Tuple, Dict

import time

import rospy
import tf

from ...datastructures.pose import Pose
from ...designators.location_designator import CostmapLocation
from ...external_interfaces.ik import request_giskard_ik
from ...external_interfaces.giskard import projection_cartesian_goal_with_approach, projection_joint_goal
from ...robot_description import ManipulatorDescription
from ...world import UseProspectionWorld, World
from ...robot_descriptions import robot_description
from ...local_transformer import LocalTransformer
from ...plan_failures import IKError
from ...costmaps import OccupancyCostmap, GaussianCostmap
from ...pose_generator_and_validator import PoseGenerator


class GiskardLocation(CostmapLocation):
    """
    Specialization version of the CostmapLocation which uses Giskard to solve for a full-body IK solution. This
    designator is especially useful for robots which lack a degree of freedom and therefore need to use the base to
    manipulate the environment effectively.
    """

    def __iter__(self) -> CostmapLocation.Location:
        """
        Uses Giskard to perform full body ik solving to get the pose of a robot at which it is able to reach a certain point.
        """
        if self.reachable_for:
            pose_right, end_config_right = self._get_reachable_pose_for_arm(self.target,
                                                                            robot_description.get_tool_frame("right"))
            pose_left, end_config_left = self._get_reachable_pose_for_arm(self.target,
                                                                          robot_description.get_tool_frame("left"))

            test_robot = World.current_world.get_prospection_object_for_object(World.robot)
            with UseProspectionWorld():
                valid, arms = reachability_validator(pose_right, test_robot, self.target, {})
                if valid:
                    yield CostmapLocation.Location(pose_right, arms)
                valid, arms = reachability_validator(pose_left, test_robot, self.target, {})
                if valid:
                    yield self.Location(pose_left, arms)

    def _get_reachable_pose_for_arm(self, target: Pose, end_effector_link: str) -> Tuple[Pose, Dict]:
        """
        :yield: An instance of CostmapLocation.Location with a pose from which the robot can reach the target
        """
        local_transformer = LocalTransformer()
        target_map = local_transformer.transform_pose(self.target, "map")

        manipulator_descs = list(
            filter(lambda chain: isinstance(chain[1], ManipulatorDescription), robot_description.chains.items()))

        near_costmap = (OccupancyCostmap(0.35, False, 200, 0.02, self.target)
                        + GaussianCostmap(200, 15, 0.02, self.target))
        for maybe_pose in PoseGenerator(near_costmap, 200):
            for name, chain in manipulator_descs:
                projection_joint_goal(robot_description.get_static_joint_chain(chain.name, "park"),
                                      allow_collisions=True)

                trajectory = projection_cartesian_goal_with_approach(maybe_pose, self.target, chain.tool_frame,
                                                                     "map", robot_description.base_link)
                last_point_positions = trajectory.trajectory.points[-1].positions
                last_point_names = trajectory.trajectory.joint_names
                last_joint_states = dict(zip(last_point_names, last_point_positions))
                orientation = list(
                    tf.transformations.quaternion_from_euler(0, 0, last_joint_states["brumbrum_yaw"], axes="sxyz"))
                pose = Pose([last_joint_states["brumbrum_x"], last_joint_states["brumbrum_y"], 0], orientation)

                robot_joint_states = {}
                for joint_name, state in last_joint_states.items():
                    if joint_name in World.robot.joints.keys():
                        robot_joint_states[joint_name] = state

                prospection_robot = World.current_world.get_prospection_object_for_object(World.robot)

                with UseProspectionWorld():
                    prospection_robot.set_joint_positions(robot_joint_states)
                    prospection_robot.set_pose(pose)
                    gripper_pose = prospection_robot.get_link_pose(chain.tool_frame)

                    if gripper_pose.dist(self.target) <= 0.02:
                        yield CostmapLocation.Location(pose, [name])

        giskard_result = achieve_cartesian_goal(target, end_effector_link, "map")
        joints = giskard_result.trajectory.joint_names
        trajectory_points = giskard_result.trajectory.points

        end_config = dict(zip(joints, trajectory_points[-1].positions))
        orientation = list(tf.transformations.quaternion_from_euler(0, 0, end_config["yaw"], axes="sxyz"))
        pose = Pose([end_config["x"], end_config["y"], 0], orientation)
        return pose, end_config