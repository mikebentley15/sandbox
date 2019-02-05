"""
This class implements forward kinematics for the LBR4 robot arm. The DH 
parameters are used to create a chain of CasADi symbolic homogeneous 
transformation matrices parameterized by the joint angles. This allows you 
to compute the end-effector pose given the joint angles of the arm.

Author: Adam Conkey
Created: Fall 2018
"""

import sys
import numpy as np
try:
    from casadi import cos, sin, SX, mtimes, Function
except ImportError:
    print("Import of casadi failed. Make sure it's installed on your system.")
    sys.exit(1)


_NUM_DOF = 7  # Number of robot arm degrees of freedom

    
class LBR4Kinematics:

    def __init__(self):
        # Symbolic joint angles
        self._thetas = SX.sym("theta", _NUM_DOF)
        # DH parameters for the LBR4 robot arm
        self._dh = [
            # a     d      alpha            theta
            [0, 0.31, 0, -np.pi / 2],
            [0, 0, -np.pi / 2, self._thetas[0]],
            [0, 0, np.pi / 2, self._thetas[1]],
            [0, 0.4, -np.pi / 2, self._thetas[2]],
            [0, 0, np.pi / 2, self._thetas[3]],
            [0, 0.39, -np.pi / 2, self._thetas[4]],
            [0, 0, np.pi / 2, self._thetas[5]],
            [0, 0.078, 0, self._thetas[6]],
        ]
        # Upper joint limits in radians
        self.upper_joint_lims = [
            2.9670,
            2.0944,
            2.9670,
            2.0944,
            2.9670,
            2.0944,
            2.9670,
        ]
        # Lower joint limits in radians
        self.lower_joint_lims = [
            -2.9670,
            -2.0944,
            -2.9670,
            -2.0944,
            -2.9670,
            -2.0944,
            -2.9670,
        ]

        # Construct the kinematic chain as a product of homogeneous TF matrices
        self._tf = SX.eye(4)
        for dh in self._dh[::-1]:
            self._tf = mtimes(self._sym_tf(*dh), self._tf)
        self._tf_func = Function('tf', [self._thetas], [self._tf], ['thetas'],
                                 ['pose'])

    def forward_kinematics(self, thetas):
        """
        Computes the end-effector pose given the robot arm's joint angles.
        Input:
            thetas : Array-like vector of joint angles (length 7)
        Output:
            Homogeneous transformation matrix expressing the end-effector pose 
            of the robot with respect to the base frame.
        """
        return self._tf_func(thetas)

    def generate_random_ee_position(self):
        """
        Generate a random valid end-effector position. 

        The procedure generates a random joint angle vector within the robot's 
        joint limits and does FK to generate a pose from which a position is 
        read. Points that are below or near the ground plane are rejected.
        """
        position = [-1] * 3
        dq = np.array(self.upper_joint_lims) - np.array(self.lower_joint_lims)
        while position[2] < 0.2:
            q = np.random.random(_NUM_DOF) * dq + np.array(self.lower_joint_lims)
            tf = self.forward_kinematics(q)
            position = self.get_translation_from_tf(tf)
        return position.full()

    def get_translation_from_tf(self, tf):
        """
        Extract the translation component of a homogeneous transformation matrix.
        """
        return tf[:3, 3]

    def get_ee_position(self, thetas):
        """
        Compute the 3-DOF end-effector position from given joint angles.
        """
        tf = self.forward_kinematics(thetas)
        position = self.get_translation_from_tf(tf)
        return position

    def _sym_tf(self, a, d, alpha, theta):
        """
        Symbolic homogeneous transformation matrix computed from DH parameters.
        """
        ct = cos(theta)
        st = sin(theta)
        ca = cos(alpha)
        sa = sin(alpha)
        tf = SX(
            np.array([[ct, -st * ca, st * sa, a * ct],
                      [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d],
                      [0, 0, 0, 1]]))
        return tf


if __name__ == '__main__':
    # Testing
    lbr4 = LBR4Kinematics()
    print "1.16", lbr4.forward_kinematics(np.ones(7) * 1.16)[:3, 3]
    print "0.67", lbr4.forward_kinematics(np.ones(7) * 0.6709)[:3, 3]
    print "0.0", lbr4.forward_kinematics(np.ones(7) * 0.0)[:3, 3]
    print "1.57", lbr4.forward_kinematics(np.ones(7) * 1.57)[:3, 3]
    #print lbr4.forward_kinematics([0, 1.57, 0, 0, 0, 0, 0])
