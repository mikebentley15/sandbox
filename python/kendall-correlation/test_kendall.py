import unittest as ut
import itertools as it
import math

import numpy as np
import scipy as sp

import kendall as K

class TestKendall(ut.TestCase):
    def setUp(self):
        rng = np.random.default_rng(42)
        self.vec = rng.uniform(high=100, size=1000)

    def test_kendalltau_abc_example(self):
        x = ['a', 'b', 'c']
        y = ['b', 'c', 'a']
        tau = K.kendalltau(x, y)
        self.assertAlmostEqual(tau.distance, 2)
        self.assertAlmostEqual(tau.normalized_distance, 2/3)
        self.assertAlmostEqual(tau.correlation, -1/3)

    def test_kendalltau_equals_scipy(self):
        N = 200
        x = self.vec[:N].copy()
        y = x.copy()
        x.sort()
        mine = K.kendalltau(x, y)
        theirs = sp.stats.kendalltau(x, y, alternative='greater')
        self.assertAlmostEqual(mine.normalized_distance, (1 - theirs.correlation) / 2)
        self.assertAlmostEqual(mine.correlation, theirs.correlation)
        self.assertAlmostEqual(mine.pvalue, theirs.pvalue)

    def test_kendalltau_and_kendalldist_agree(self):
        N = 30
        vec = self.vec[:N].copy()
        tau = K.kendalltau(vec)
        tauD = K.kendalltau(vec, dist='default')
        dist = K.kendalldist(vec)
        distD = K.kendalldist(vec, dist='default')
        self.assertAlmostEqual(tau.distance, dist)
        self.assertAlmostEqual(tauD.distance, distD)

    def test_kendalltau_and_kendalltauD_agree(self):
        N = 30
        vec = self.vec[:N].copy()
        D = np.ones((N,N))
        tau = K.kendalltau(vec)
        tauD = K.kendalltau(vec, D=D)
        tau2 = K.kendalltau(vec * 3)
        tauD2 = K.kendalltau(vec * 3, D=5*D)
        self.assertAlmostEqual(tau.distance, tauD.distance)
        self.assertAlmostEqual(tau.distance, tau2.distance)
        self.assertAlmostEqual(tau.distance, tauD2.distance / 5)
        self.assertAlmostEqual(tau.normalized_distance, tau2.normalized_distance)
        self.assertAlmostEqual(tau.normalized_distance, tauD.normalized_distance)
        self.assertAlmostEqual(tau.normalized_distance, tauD2.normalized_distance)
        self.assertAlmostEqual(tau.correlation, tauD.correlation)
        self.assertAlmostEqual(tau.correlation, tau2.correlation)
        self.assertAlmostEqual(tau.correlation, tauD2.correlation)
        self.assertAlmostEqual(tau.zscore, tauD.zscore)
        self.assertAlmostEqual(tau.zscore, tau2.zscore)
        self.assertAlmostEqual(tau.zscore, tauD2.zscore)
        self.assertAlmostEqual(tau.pvalue, tauD.pvalue)
        self.assertAlmostEqual(tau.pvalue, tau2.pvalue)
        self.assertAlmostEqual(tau.pvalue, tauD2.pvalue)

    def test_kendalltauDist_exact_pvalue_size6(self):
        'Compare z-score estimated p-value against an exact calculation'
        # p-value = p_null(tau_dist <= observed)
        N = 6
        vec = self.vec[:N].copy()
        mine = K.kendalltau(vec, dist='default')
        better_distance_count = sum(
            mine.distance >= K.kendalldist(x, dist='default')
            for x in it.permutations(vec)
        )
        exact_pvalue = better_distance_count / math.factorial(N)
        self.assertAlmostEqual(mine.pvalue, exact_pvalue, delta=0.02)

    def test_QD2_estimated_vs_calculated_for_simple_D(self):
        N = 30
        nsamples = 600
        x = self.vec[:N].copy()
        unsrt = np.argsort(x)
        srt = unsrt.copy()
        srt.sort()
        D = K._distmat(srt, assume_sorted=True)
        Dsum2 = np.sum(D)**2
        estimated = K._estimated_expected_QD2(D, nsamples=nsamples) / Dsum2
        calculated = K._expected_QD2(D) / Dsum2
        self.assertAlmostEqual(estimated, calculated, places=2)

    def test_QD2_estimated_vs_calculated(self):
        N = 30
        nsamples = 600
        x = self.vec[:N].copy()
        unsrt = np.argsort(x)
        srt = unsrt.copy()
        srt.sort()
        D = K._distmat(srt, assume_sorted=True)
        Dsum2 = np.sum(D)**2
        estimated = K._estimated_expected_QD2(D, nsamples=nsamples) / Dsum2
        calculated = K._expected_QD2(D) / Dsum2
        self.assertAlmostEqual(estimated, calculated, places=2)

    def test_permutation_probability(self):
        N = 5
        ordered = np.arange(N)
        expected_Sigma_ijkl = np.zeros((N,N,N,N))
        for P in it.permutations(ordered):
            S = K._swapmat(ordered, P)
            for j in range(1, N):
                for i in range(j):
                    expected_Sigma_ijkl[i,j,:,:] += S[i,j] * S
        expected_Sigma_ijkl /= math.factorial(N)
        actual_Sigma_ijkl = np.zeros((N,N,N,N))
        for j, L in it.product(range(1, N), range(1, N)):
            for i, k in it.product(range(j), range(L)):
                if i == k and j == L:
                    p = 0.5
                elif i == k or j == L:
                    p = 1./3.
                elif i == L or j == k:
                    p = 1./6.
                else:
                    p = 0.25
                actual_Sigma_ijkl[i,j,k,L] = p
        np.testing.assert_array_almost_equal(expected_Sigma_ijkl, actual_Sigma_ijkl)

if __name__ == '__main__':
    ut.main()
