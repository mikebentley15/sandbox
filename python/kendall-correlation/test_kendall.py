import unittest as ut

import numpy as np
import scipy as sp

import kendall as K

class TestKendall(ut.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng()

    def test_kendalltau_abc_example(self):
        x = ['a', 'b', 'c']
        y = ['b', 'c', 'a']
        tau = K.kendalltau(x, y)
        self.assertAlmostEqual(tau.distance, 2)
        self.assertAlmostEqual(tau.normalized_distance, 2/3)
        self.assertAlmostEqual(tau.correlation, -1/3)

    def test_kendalltau_equals_scipy(self):
        N = 200
        x = self.rng.uniform(high=100, size=N)
        y = x.copy()
        x.sort()
        mine = K.kendalltau(x, y)
        theirs = sp.stats.kendalltau(x, y, alternative='greater')
        self.assertAlmostEqual(mine.normalized_distance, (1 - theirs.correlation) / 2)
        self.assertAlmostEqual(mine.correlation, theirs.correlation)
        self.assertAlmostEqual(mine.pvalue, theirs.pvalue)

if __name__ == '__main__':
    ut.main()
