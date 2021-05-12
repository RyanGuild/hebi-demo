from typing import List, Optional
from hebi import robot_model, GroupFeedback, GroupCommand
from time import sleep
from hebi._internal.group import Group
import numpy as np
from numpy import ndarray
from util import get_lookup

class Robit:
    control_group: Group
    model: robot_model.RobotModel
    current_position: Optional[ndarray] = None
    current_end_effector: Optional[ndarray] = None

    def __init__(self, family_name: str, module_names: List[str], model_name="arm-model.hrdf", feedback_hz=10.0):
        lookup = get_lookup()

        try:
            self.model = robot_model.import_from_hrdf("arm-model.hrdf")
        except:
            print("Could not load HRDF.")
            exit(1)

        self.control_group = lookup.get_group_from_names(
            [family_name], module_names)

        if self.control_group is None:
            print(
                'Group not found: Did you forget to set the module family and name above?')
            exit(1)

        def feedback_handler(group_fbk: GroupFeedback):
            self.current_position = group_fbk.position
            self.current_end_effector = self.model.get_end_effector(self.current_position)


        self.control_group.add_feedback_handler(feedback_handler)
        self.control_group.feedback_frequency = feedback_hz

    def __sizeof__(self) -> int:
        return self.control_group.size

    def __str__(self) -> str:
        return "Robit(\n{},\n {})".format(self.control_group, self.model)

    def get_grav_comp_efforts(self, positions, gravityVec):
        # Normalize gravity vector (to 1g, or 9.8 m/s^2)
        normed_gravity = gravityVec / np.linalg.norm(gravityVec) * 9.81

        jacobians = self.model.get_jacobians('CoM', positions)
        # Get torque for each module
        # comp_torque = J' * wrench_vector
        # (for each frame, sum this quantity)
        comp_torque = np.zeros((self.model.dof_count, 1))

        # Wrench vector
        wrench_vec = np.zeros(6)  # For a single frame; this is (Fx/y/z, tau x/y/z)
        num_frames = self.model.get_frame_count('CoM')

        for i in range(num_frames):
            # Add the torques for each joint to support the mass at this frame
            wrench_vec[0:3] = normed_gravity * self.model.masses[i]
            comp_torque += jacobians[i].transpose() * np.reshape(wrench_vec, (6, 1))

        return np.squeeze(comp_torque)

    def go_to_target(self, theta: float, x: float, y:float):

        if not self.internal_sphere_check(theta, x, y):
            return False
        
        if not self.low_plane_check(theta, x, y):
            return False

        target = robot_model.endeffector_position_objective([theta, x, y])
        angles = self.model.solve_inverse_kinematics(self.current_position, target)
        group_command = GroupCommand(self.control_group.size)
        group_command.position = angles

        for i in range(100):
            self.control_group.send_command(group_command)
            # Note: the arm will go limp after the 100 ms command lifetime,
            # so the command is repeated every 50 ms for 5 seconds.
            sleep(0.05)

    def internal_sphere_check(self, theta: float, x: float, y:float) -> bool:
        if np.sqrt(x*x + y*y) < .40:
            return False
        return True

    def low_plane_check(self, theta: float, x: float, y:float) -> bool:
        if y < .1:
            return False
        return True

