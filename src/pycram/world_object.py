import logging
import os

import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion
from typing_extensions import Type, Optional, Dict, Tuple, List, Union

from pycram.enums import ObjectType, JointType
from pycram.local_transformer import LocalTransformer
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.world import WorldEntity, World
from pycram.world_constraints import Attachment
from pycram.world_dataclasses import Color, ObjectState, LinkState, JointState, AxisAlignedBoundingBox


class Object(WorldEntity):
    """
    Represents a spawned Object in the World.
    """

    prospection_world_prefix: str = "prospection/"
    """
    The ObjectDescription of the object, this contains the name and type of the object as well as the path to the source 
    file.
    """

    def __init__(self, name: str, obj_type: ObjectType, path: str,
                 description: Type['ObjectDescription'],
                 pose: Optional[Pose] = None,
                 world: Optional[World] = None,
                 color: Optional[Color] = Color(),
                 ignore_cached_files: Optional[bool] = False):
        """
        The constructor loads the description file into the given World, if no World is specified the
        :py:attr:`~World.current_world` will be used. It is also possible to load .obj and .stl file into the World.
        The rgba_color parameter is only used when loading .stl or .obj files,
         for URDFs :func:`~Object.set_color` can be used.

        :param name: The name of the object
        :param obj_type: The type of the object as an ObjectType enum.
        :param path: The path to the source file, if only a filename is provided then the resources directories will be
         searched.
        :param description: The ObjectDescription of the object, this contains the joints and links of the object.
        :param pose: The pose at which the Object should be spawned
        :param world: The World in which the object should be spawned,
         if no world is specified the :py:attr:`~World.current_world` will be used.
        :param color: The rgba_color with which the object should be spawned.
        :param ignore_cached_files: If true the file will be spawned while ignoring cached files.
        """

        super().__init__(-1, world)

        if pose is None:
            pose = Pose()

        self.name: str = name
        self.obj_type: ObjectType = obj_type
        self.color: Color = color
        self.description = description()
        self.cache_manager = self.world.cache_manager

        self.local_transformer = LocalTransformer()
        self.original_pose = self.local_transformer.transform_pose(pose, "map")
        self._current_pose = self.original_pose

        self.id, self.path = self._load_object_and_get_id(path, ignore_cached_files)

        self.description.update_description_from_file(self.path)

        self.tf_frame = ((self.prospection_world_prefix if self.world.is_prospection_world else "")
                         + f"{self.name}_{self.id}")

        self._update_object_description_from_file(self.path)

        if self.description.name == robot_description.name:
            self.world.set_robot_if_not_set(self)

        self._init_joint_name_and_id_map()
        self._init_link_name_and_id_map()

        self._init_links_and_update_transforms()
        self._init_joints()

        self.attachments: Dict[Object, Attachment] = {}

        if not self.world.is_prospection_world:
            self._add_to_world_sync_obj_queue()

        self.world.objects.append(self)

    def _load_object_and_get_id(self, path, ignore_cached_files: bool) -> Tuple[int, str]:
        """
        Loads an object to the given World with the given position and orientation. The rgba_color will only be
        used when an .obj or .stl file is given.
        If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
        and this URDf file will be loaded instead.
        When spawning a URDf file a new file will be created in the cache directory, if there exists none.
        This new file will have resolved mesh file paths, meaning there will be no references
        to ROS packges instead there will be absolute file paths.

        :param ignore_cached_files: Whether to ignore files in the cache directory.
        :return: The unique id of the object and the path of the file that was loaded.
        """

        path = self.world.update_cache_dir_with_object(path, ignore_cached_files, self)

        try:
            obj_id = self.world.load_description_and_get_object_id(path, Pose(self.get_position_as_list(),
                                                                              self.get_orientation_as_list()))
            return obj_id, path
        except Exception as e:
            logging.error(
                "The File could not be loaded. Please note that the path has to be either a URDF, stl or obj file or"
                " the name of an URDF string on the parameter server.")
            os.remove(path)
            raise (e)

    def _update_object_description_from_file(self, path: str) -> None:
        """
        Updates the object description from the given file path.
        :param path: The path to the file from which the object description should be updated.
        """
        self.description.update_description_from_file(path)

    def _init_joint_name_and_id_map(self) -> None:
        """
        Creates a dictionary which maps the joint names to their unique ids and vice versa.
        """
        n_joints = len(self.joint_names)
        self.joint_name_to_id = dict(zip(self.joint_names, range(n_joints)))
        self.joint_id_to_name = dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))

    def _init_link_name_and_id_map(self) -> None:
        """
        Creates a dictionary which maps the link names to their unique ids and vice versa.
        """
        n_links = len(self.link_names)
        self.link_name_to_id: Dict[str, int] = dict(zip(self.link_names, range(n_links)))
        self.link_name_to_id[self.description.get_root()] = -1
        self.link_id_to_name: Dict[int, str] = dict(zip(self.link_name_to_id.values(), self.link_name_to_id.keys()))

    def _init_links_and_update_transforms(self) -> None:
        """
        Initializes the link objects from the URDF file and creates a dictionary which maps the link names to the
        corresponding link objects.
        """
        self.links = {}
        for link_name, link_id in self.link_name_to_id.items():
            link_description = self.description.get_link_by_name(link_name)
            if link_name == self.description.get_root():
                self.links[link_name] = self.description.RootLink(self)
            else:
                self.links[link_name] = self.description.Link(link_id, link_description, self)

        self.update_link_transforms()

    def _init_joints(self):
        """
        Initialize the joint objects from the URDF file and creates a dictionary which mas the joint names to the
        corresponding joint objects
        """
        self.joints = {}
        for joint_name, joint_id in self.joint_name_to_id.items():
            joint_description = self.description.get_joint_by_name(joint_name)
            self.joints[joint_name] = self.description.Joint(joint_id, joint_description, self)

    def _add_to_world_sync_obj_queue(self) -> None:
        """
        Adds this object to the objects queue of the WorldSync object of the World.
        """
        self.world.world_sync.add_obj_queue.put(
            [self.name, self.obj_type, self.path, type(self.description), self.get_position_as_list(),
             self.get_orientation_as_list(),
             self.world.prospection_world, self.color, self])

    @property
    def link_names(self) -> List[str]:
        """
        :return: The name of each link as a list.
        """
        return self.world.get_object_link_names(self)

    @property
    def number_of_links(self) -> int:
        """
        :return: The number of links of this object.
        """
        return len(self.description.links)

    @property
    def joint_names(self) -> List[str]:
        """
        :return: The name of each joint as a list.
        """
        return self.world.get_object_joint_names(self)

    @property
    def number_of_joints(self) -> int:
        """
        :return: The number of joints of this object.
        """
        return len(self.description.joints)

    @property
    def base_origin_shift(self) -> np.ndarray:
        """
        The shift between the base of the object and the origin of the object.
        :return: A numpy array with the shift between the base of the object and the origin of the object.
        """
        return np.array(self.get_pose().position_as_list()) - np.array(self.get_base_origin().position_as_list())

    def __repr__(self):
        skip_attr = ["links", "joints", "description", "attachments"]
        return self.__class__.__qualname__ + f"(" + ', \n'.join(
            [f"{key}={value}" if key not in skip_attr else f"{key}: ..." for key, value in self.__dict__.items()]) + ")"

    def remove(self) -> None:
        """
        Removes this object from the World it currently resides in.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to world remove object is done
        to remove this Object from the simulation/world.
        """
        self.world.remove_object(self)

    def reset(self, remove_saved_states=True) -> None:
        """
        Resets the Object to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and the object will be set to the position and
        orientation in which it was spawned.
        :param remove_saved_states: If True the saved states will be removed.
        """
        self.detach_all()
        self.reset_all_joints_positions()
        self.set_pose(self.original_pose)
        if remove_saved_states:
            self.remove_saved_states()

    def attach(self,
               child_object: 'Object',
               parent_link: Optional[str] = None,
               child_link: Optional[str] = None,
               bidirectional: Optional[bool] = True) -> None:
        """
        Attaches another object to this object. This is done by
        saving the transformation between the given link, if there is one, and
        the base pose of the other object. Additionally, the name of the link, to
        which the object is attached, will be saved.
        Furthermore, a simulator constraint will be created so the attachment
        also works while simulation.
        Loose attachments means that the attachment will only be one-directional. For example, if this object moves the
        other, attached, object will also move but not the other way around.

        :param child_object: The other object that should be attached.
        :param parent_link: The link name of this object.
        :param child_link: The link name of the other object.
        :param bidirectional: If the attachment should be a loose attachment.
        """
        parent_link = self.links[parent_link] if parent_link else self.root_link
        child_link = child_object.links[child_link] if child_link else child_object.root_link

        attachment = Attachment(parent_link, child_link, bidirectional)

        self.attachments[child_object] = attachment
        child_object.attachments[self] = attachment.get_inverse()

        self.world.attachment_event(self, [self, child_object])

    def detach(self, child_object: 'Object') -> None:
        """
        Detaches another object from this object. This is done by
        deleting the attachment from the attachments dictionary of both objects
        and deleting the constraint of the simulator.
        Afterward the detachment event of the corresponding World will be fired.

        :param child_object: The object which should be detached
        """
        del self.attachments[child_object]
        del child_object.attachments[self]

        self.world.detachment_event(self, [self, child_object])

    def detach_all(self) -> None:
        """
        Detach all objects attached to this object.
        """
        attachments = self.attachments.copy()
        for att in attachments.keys():
            self.detach(att)

    def update_attachment_with_object(self, child_object: 'Object'):
        self.attachments[child_object].update_transform_and_constraint()

    def get_position(self) -> Point:
        """
        Returns the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_pose().position

    def get_orientation(self) -> Pose.orientation:
        """
        Returns the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return self.get_pose().orientation

    def get_position_as_list(self) -> List[float]:
        """
        Returns the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_pose().position_as_list()

    def get_orientation_as_list(self) -> List[float]:
        """
        Returns the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return self.get_pose().orientation_as_list()

    def get_pose(self) -> Pose:
        """
        Returns the position of this object as a list of xyz. Alias for :func:`~Object.get_position`.

        :return: The current pose of this object
        """
        return self._current_pose

    def set_pose(self, pose: Pose, base: Optional[bool] = False, set_attachments: Optional[bool] = True) -> None:
        """
        Sets the Pose of the object.

        :param pose: New Pose for the object
        :param base: If True places the object base instead of origin at the specified position and orientation
        :param set_attachments: Whether to set the poses of the attached objects to this object or not.
        """
        pose_in_map = self.local_transformer.transform_pose(pose, "map")
        if base:
            pose_in_map.position = np.array(pose_in_map.position) + self.base_origin_shift

        self.reset_base_pose(pose_in_map)

        if set_attachments:
            self._set_attached_objects_poses()

    def reset_base_pose(self, pose: Pose):
        self.world.reset_object_base_pose(self, pose)
        self.update_pose()

    def update_pose(self):
        """
        Updates the current pose of this object from the world, and updates the poses of all links.
        """
        self._current_pose = self.world.get_object_pose(self)
        self._update_all_links_poses()
        self.update_link_transforms()

    def _update_all_joints_positions(self):
        """
        Updates the posisitons of all joints by getting them from the simulator.
        """
        for joint in self.joints.values():
            joint._update_position()

    def _update_all_links_poses(self):
        """
        Updates the poses of all links by getting them from the simulator.
        """
        for link in self.links.values():
            link._update_pose()

    def move_base_to_origin_pos(self) -> None:
        """
        Move the object such that its base will be at the current origin position.
        This is useful when placing objects on surfaces where you want the object base in contact with the surface.
        """
        self.set_pose(self.get_pose(), base=True)

    def save_state(self, state_id) -> None:
        """
        Saves the state of this object by saving the state of all links and attachments.
        :param state_id: The unique id of the state.
        """
        self.save_links_states(state_id)
        self.save_joints_states(state_id)
        super().save_state(state_id)

    def save_links_states(self, state_id: int) -> None:
        """
        Saves the state of all links of this object.
        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.save_state(state_id)

    def save_joints_states(self, state_id: int) -> None:
        """
        Saves the state of all joints of this object.
        :param state_id: The unique id of the state.
        """
        for joint in self.joints.values():
            joint.save_state(state_id)

    @property
    def current_state(self) -> ObjectState:
        return ObjectState(self.attachments.copy(), self.link_states, self.joint_states)

    @current_state.setter
    def current_state(self, state: ObjectState) -> None:
        self.attachments = state.attachments
        self.link_states = state.link_states
        self.joint_states = state.joint_states

    @property
    def link_states(self) -> Dict[int, LinkState]:
        """
        Returns the current state of all links of this object.
        :return: A dictionary with the link id as key and the current state of the link as value.
        """
        return {link.id: link.current_state for link in self.links.values()}

    @link_states.setter
    def link_states(self, link_states: Dict[int, LinkState]) -> None:
        """
        Sets the current state of all links of this object.
        :param link_states: A dictionary with the link id as key and the current state of the link as value.
        """
        for link in self.links.values():
            link.current_state = link_states[link.id]

    @property
    def joint_states(self) -> Dict[int, JointState]:
        """
        Returns the current state of all joints of this object.
        :return: A dictionary with the joint id as key and the current state of the joint as value.
        """
        return {joint.id: joint.current_state for joint in self.joints.values()}

    @joint_states.setter
    def joint_states(self, joint_states: Dict[int, JointState]) -> None:
        """
        Sets the current state of all joints of this object.
        :param joint_states: A dictionary with the joint id as key and the current state of the joint as value.
        """
        for joint in self.joints.values():
            joint.current_state = joint_states[joint.id]

    def restore_state(self, state_id: int) -> None:
        """
        Restores the state of this object by restoring the state of all links and attachments.
        :param state_id: The unique id of the state.
        """
        self.restore_attachments(state_id)
        self.restore_links_states(state_id)
        self.restore_joints_states(state_id)

    def restore_attachments(self, state_id: int) -> None:
        """
        Restores the attachments of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        self.attachments = self.saved_states[state_id].attachments

    def restore_links_states(self, state_id: int) -> None:
        """
        Restores the states of all links of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.restore_state(state_id)

    def restore_joints_states(self, state_id: int) -> None:
        """
        Restores the states of all joints of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        for joint in self.joints.values():
            joint.restore_state(state_id)

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of this object.
        """
        super().remove_saved_states()
        self.remove_links_saved_states()
        self.remove_joints_saved_states()

    def remove_links_saved_states(self) -> None:
        """
        Removes all saved states of the links of this object.
        """
        for link in self.links.values():
            link.remove_saved_states()

    def remove_joints_saved_states(self) -> None:
        """
        Removes all saved states of the joints of this object.
        """
        for joint in self.joints.values():
            joint.remove_saved_states()

    def _set_attached_objects_poses(self, already_moved_objects: Optional[List['Object']] = None) -> None:
        """
        Updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param already_moved_objects: A list of Objects that were already moved, these will be excluded to prevent
         loops in the update.
        """

        if already_moved_objects is None:
            already_moved_objects = []

        for child in self.attachments:

            if child in already_moved_objects:
                continue

            attachment = self.attachments[child]
            if not attachment.bidirectional:
                self.update_attachment_with_object(child)
                child.update_attachment_with_object(self)

            else:
                link_to_object = attachment.parent_to_child_transform
                child.set_pose(link_to_object.to_pose(), set_attachments=False)
                child._set_attached_objects_poses(already_moved_objects + [self])

    def set_position(self, position: Union[Pose, Point], base=False) -> None:
        """
        Sets this Object to the given position, if base is true the bottom of the Object will be placed at the position
        instead of the origin in the center of the Object. The given position can either be a Pose,
         in this case only the position is used or a geometry_msgs.msg/Point which is the position part of a Pose.

        :param position: Target position as xyz.
        :param base: If the bottom of the Object should be placed or the origin in the center.
        """
        pose = Pose()
        if isinstance(position, Pose):
            target_position = position.position
            pose.frame = position.frame
        elif isinstance(position, Point):
            target_position = position
        elif isinstance(position, list):
            target_position = position
        else:
            raise TypeError("The given position has to be a Pose, Point or a list of xyz.")

        pose.position = target_position
        pose.orientation = self.get_orientation()
        self.set_pose(pose, base=base)

    def set_orientation(self, orientation: Union[Pose, Quaternion]) -> None:
        """
        Sets the orientation of the Object to the given orientation. Orientation can either be a Pose, in this case only
        the orientation of this pose is used or a geometry_msgs.msg/Quaternion which is the orientation of a Pose.

        :param orientation: Target orientation given as a list of xyzw.
        """
        pose = Pose()
        if isinstance(orientation, Pose):
            target_orientation = orientation.orientation
            pose.frame = orientation.frame
        else:
            target_orientation = orientation

        pose.pose.position = self.get_position()
        pose.pose.orientation = target_orientation
        self.set_pose(pose)

    def get_joint_id(self, name: str) -> int:
        """
        Returns the unique id for a joint name. As used by the world/simulator.

        :param name: The joint name
        :return: The unique id
        """
        return self.joint_name_to_id[name]

    def get_root_link_description(self) -> 'LinkDescription':
        """
        Returns the root link of the URDF of this object.
        :return: The root link as defined in the URDF of this object.
        """
        root_link_name = self.description.get_root()
        for link_description in self.description.links:
            if link_description.name == root_link_name:
                return link_description

    @property
    def root_link(self) -> 'Link':
        """
        Returns the root link of this object.
        :return: The root link of this object.
        """
        return self.links[self.description.get_root()]

    @property
    def root_link_name(self) -> str:
        """
        Returns the name of the root link of this object.
        :return: The name of the root link of this object.
        """
        return self.description.get_root()

    def get_root_link_id(self) -> int:
        """
        Returns the unique id of the root link of this object.
        :return: The unique id of the root link of this object.
        """
        return self.get_link_id(self.description.get_root())

    def get_link_id(self, link_name: str) -> int:
        """
        Returns a unique id for a link name.
        :param link_name: The name of the link.
        :return: The unique id of the link.
        """
        return self.link_name_to_id[link_name]

    def get_link_by_id(self, link_id: int) -> 'Link':
        """
        Returns the link for a given unique link id
        :param link_id: The unique id of the link.
        :return: The link object.
        """
        return self.links[self.link_id_to_name[link_id]]

    def get_joint_by_id(self, joint_id: int) -> str:
        """
        Returns the joint name for a unique world id

        :param joint_id: The world id of for joint
        :return: The joint name
        """
        return dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))[joint_id]

    def reset_all_joints_positions(self) -> None:
        """
        Sets the current position of all joints to 0. This is useful if the joints should be reset to their default
        """
        joint_names = list(self.joint_name_to_id.keys())
        joint_positions = [0] * len(joint_names)
        self.set_joint_positions(dict(zip(joint_names, joint_positions)))

    def set_joint_positions(self, joint_poses: dict) -> None:
        """
        Sets the current position of multiple joints at once, this method should be preferred when setting
         multiple joints at once instead of running :func:`~Object.set_joint_position` in a loop.

        :param joint_poses:
        """
        for joint_name, joint_position in joint_poses.items():
            self.joints[joint_name].position = joint_position
        # self.update_pose()
        self._update_all_links_poses()
        self.update_link_transforms()
        self._set_attached_objects_poses()

    def set_joint_position(self, joint_name: str, joint_position: float) -> None:
        """
        Sets the position of the given joint to the given joint pose and updates the poses of all attached objects.

        :param joint_name: The name of the joint
        :param joint_position: The target pose for this joint
        """
        self.joints[joint_name].position = joint_position
        self._update_all_links_poses()
        self.update_link_transforms()
        self._set_attached_objects_poses()

    def get_joint_position(self, joint_name: str) -> float:
        return self.joints[joint_name].position

    def get_joint_damping(self, joint_name: str) -> float:
        return self.joints[joint_name].damping

    def get_joint_upper_limit(self, joint_name: str) -> float:
        return self.joints[joint_name].upper_limit

    def get_joint_lower_limit(self, joint_name: str) -> float:
        return self.joints[joint_name].lower_limit

    def get_joint_axis(self, joint_name: str) -> Point:
        return self.joints[joint_name].axis

    def get_joint_type(self, joint_name: str) -> JointType:
        return self.joints[joint_name].type

    def get_joint_limits(self, joint_name: str) -> Tuple[float, float]:
        return self.joints[joint_name].limits

    def find_joint_above(self, link_name: str, joint_type: JointType) -> str:
        """
        Traverses the chain from 'link' to the URDF origin and returns the first joint that is of type 'joint_type'.

        :param link_name: AbstractLink name above which the joint should be found
        :param joint_type: Joint type that should be searched for
        :return: Name of the first joint which has the given type
        """
        chain = self.description.get_chain(self.description.get_root(), link_name)
        reversed_chain = reversed(chain)
        container_joint = None
        for element in reversed_chain:
            if element in self.joint_name_to_id and self.get_joint_type(element) == joint_type:
                container_joint = element
                break
        if not container_joint:
            rospy.logwarn(f"No joint of type {joint_type} found above link {link_name}")
        return container_joint

    def get_positions_of_all_joints(self) -> Dict[str, float]:
        """
        Returns the positions of all joints of the object as a dictionary of joint names and joint positions.

        :return: A dictionary with all joints positions'.
        """
        return {j.name: j.position for j in self.joints.values()}

    def update_link_transforms(self, transform_time: Optional[rospy.Time] = None) -> None:
        """
        Updates the transforms of all links of this object using time 'transform_time' or the current ros time.
        """
        for link in self.links.values():
            link.update_transform(transform_time)

    def contact_points(self) -> List:
        """
        Returns a list of contact points of this Object with other Objects.

        :return: A list of all contact points with other objects
        """
        return self.world.get_object_contact_points(self)

    def contact_points_simulated(self) -> List:
        """
        Returns a list of all contact points between this Object and other Objects after stepping the simulation once.

        :return: A list of contact points between this Object and other Objects
        """
        state_id = self.world.save_state()
        self.world.step()
        contact_points = self.contact_points()
        self.world.restore_state(state_id)
        return contact_points

    def set_color(self, rgba_color: Color) -> None:
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values.

        :param rgba_color: The color as Color object with RGBA values between 0 and 1
        """
        # Check if there is only one link, this is the case for primitive
        # forms or if loaded from an .stl or .obj file
        if self.links != {}:
            for link in self.links.values():
                link.color = rgba_color
        else:
            self.root_link.color = rgba_color

    def get_color(self) -> Union[Color, Dict[str, Color]]:
        """
        This method returns the rgba_color of this object. The return is either:

            1. A Color object with RGBA values, this is the case if the object only has one link (this
                happens for example if the object is spawned from a .obj or .stl file)
            2. A dict with the link name as key and the rgba_color as value. The rgba_color is given as a Color Object.
                Please keep in mind that not every link may have a rgba_color. This is dependent on the URDF from which
                 the object is spawned.

        :return: The rgba_color as Color object with RGBA values between 0 and 1 or a dict with the link name as key and
            the rgba_color as value.
        """
        link_to_color_dict = self.links_colors

        if len(link_to_color_dict) == 1:
            return list(link_to_color_dict.values())[0]
        else:
            return link_to_color_dict

    @property
    def links_colors(self) -> Dict[str, Color]:
        """
        The color of each link as a dictionary with link names as keys and RGBA colors as values.
        """
        return self.world.get_colors_of_object_links(self)

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis aligned bounding box of this object.
        """
        return self.world.get_object_axis_aligned_bounding_box(self)

    def get_base_origin(self) -> Pose:
        """
        :return: the origin of the base/bottom of this object.
        """
        aabb = self.get_link_by_id(-1).get_axis_aligned_bounding_box()
        base_width = np.absolute(aabb.min_x - aabb.max_x)
        base_length = np.absolute(aabb.min_y - aabb.max_y)
        return Pose([aabb.min_x + base_width / 2, aabb.min_y + base_length / 2, aabb.min_z],
                    self.get_pose().orientation_as_list())

    def __copy__(self) -> 'Object':
        """
        Returns a copy of this object. The copy will have the same name, type, path, description, pose, world and color.
        :return: A copy of this object.
        """
        return Object(self.name, self.obj_type, self.path, type(self.description), self.get_pose(),
                      self.world.prospection_world, self.color)
