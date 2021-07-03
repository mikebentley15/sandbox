#!/usr/bin/env python3
'''
Functionality to align two vectors together.

Provides the function vecalign(a, b) that takes two 3D vectors and returns a
rotation matrix that transforms b into a.

When this script is run directly, it runs its unit tests
'''

import numpy as np
from math import acos

def vecalign(a, b):
    return _vecalign_mine(a, b)

def _vecalign_mine(a, b):
    '''
    Returns the rotation matrix that can rotate the 3 dimensional b vector to
    be aligned with the a vector.

    @param a (3-dim array-like): destination vector
    @param b (3-dim array-like): vector to rotate align with a in direction

    the vectors a and b do not need to be normalized.  They can be column
    vectors or row vectors

    Much of the logic of this function came from Eigen3, but it's the way I
    would have done it too; find the angle between the two vectors, get one
    vector that is mutually perpendicular to the two given vectors, and then do
    angle-axis about that perpendicular vector.
    '''
    with np.errstate(divide='raise', under='raise', over='raise', invalid='raise'):
        a = np.asarray(a)
        b = np.asarray(b)
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        cos_theta = a.dot(b) # since a and be are unit vecs, a.dot(b) = cos(theta)

        # if dot is close to 1, vectors are nearly equal
        if np.isclose(cos_theta, 1):
            # TODO: solve better than just assuming they are exactly identical
            return np.eye(3)

        # if dot is close to -1, vectors are nearly opposites
        if np.isclose(cos_theta, -1):
            # TODO: solve better than just assuming they are exactly opposite
            return -np.eye(3)

        axis = np.cross(b, a)
        sin_theta = np.linalg.norm(axis)
        axis = axis / sin_theta
        c = cos_theta
        s = sin_theta
        t = 1 - c
        x = axis[0]
        y = axis[1]
        z = axis[2]

        # angle-axis formula to create a rotation matrix
        return np.array([
            [t*x*x + c  , t*x*y - z*s, t*x*z + y*s],
            [t*x*y + z*s, t*y*y + c  , t*y*z - x*s],
            [t*x*z - y*s, t*y*z + x*s, t*z*z + c  ],
            ])

if __name__ == '__main__':
    # define unit tests
    import unittest as ut

    class VecalignTest(ut.TestCase):
        def assert_aligns(self, a, b, rot):
            a = np.asarray(a)
            b = np.asarray(b)
            a = a / np.linalg.norm(a) # normalize
            b = b / np.linalg.norm(b) # normalize
            np.testing.assert_allclose(rot.dot(b), a)

        def test_same_vec(self):
            a = [1, 2, 3]
            b = [1, 2, 3]
            self.assert_aligns(a, b, vecalign(a, b))

        def test_opposite_vec(self):
            a = [ 1,  1,  1]
            b = [-1, -1, -1]
            self.assert_aligns(a, b, vecalign(a, b))

        def test_zero_vec(self):
            with self.assertRaises(FloatingPointError):
                a = [0, 0, 0]
                b = [1, 2, 3]
                vecalign(a, b)
            with self.assertRaises(FloatingPointError):
                a = [1, 2, 3]
                b = [0, 0, 0]
                vecalign(a, b)

        def test_x_to_y(self):
            a = [1, 0, 0]
            b = [0, 1, 0]
            self.assert_aligns(a, b, vecalign(a, b))

        def test_x_to_z(self):
            a = [1, 0, 0]
            b = [0, 0, 1]
            self.assert_aligns(a, b, vecalign(a, b))

        def test_chosen_vecs(self):
            a = [0.20545724, 0.5580496 , 0.58713717]
            b = [0.22801554, 0.68495085, 0.6900671 ]
            self.assert_aligns(a, b, vecalign(a, b))

        def test_random_vecs(self):
            a = np.random.random(3)
            b = np.random.random(3)
            self.assert_aligns(a, b, vecalign(a, b))

    ut.main()
