from collections import namedtuple
import itertools as it

import numpy as np
import scipy

KendallTau = namedtuple('KendallTau', [
    'distance',
    'normalized_distance',
    'correlation',
    'zscore',
    'pvalue',
])

def kendalltau(x, y=None, dist=None, D=None):
    '''
    Calculate the Kendall Tau distance and correlation coefficient with p-value

    @param x: array
        If y is None, then x is compared against sorted(x), else x is
        considered the "sorted" version, and y is the "unsorted" version
        compared against x.
    @param y: array
        If specified, measure tau of this against order of x.
    @param dist: str or function
        Distance metric to use.  Can specify 'default' to use abs(b-a) as the
        distance function.  If given a function, will be called with parameters
        (a, b) from x where b > a (according to the orderings of x).
    @param D: matrix
        Distance metric matrix (only upper triangle is used).  This is in replacement of the dist function (i.e., use entries from this matrix instead of the dist function).  The i,j entry means dist(x[j] - x[i]).

    @return KendallTau object fully filled out
    '''
    S, D = _kendallprep(x, y, dist, D)
    if D is None:
        distance, normed_dist, cor_var = _normal_kendall_tau(S)
    else:
        distance, normed_dist, cor_var = _general_kendall_tau(S, D)
    cor = 1 - 2 * normed_dist
    zscore = cor / np.sqrt(cor_var)
    pvalue = _zscore_to_pvalue(zscore, tail='right')
    return KendallTau(distance, normed_dist, cor, zscore, pvalue)

def kendalldist(x, y=None, dist=None, D=None):
    '''Calculate just the Kendall distance (unnormalized).  Similar to kendalltau()
    Returns just the distance.
    '''
    S, D = _kendallprep(x, y, dist, D)
    if D is None:
        return np.sum(S)
    else:
        return np.sum(S * D)

def _kendallprep(x, y, dist, D):
    'Calc and return (S, D) from kendalldist() or kendalltau() args'
    # populate srt and usrt for sorted and unsorted arrays, respectively
    x = np.asarray(x)
    if y is None:
        srt, usrt = x.copy(), x
        srt.sort()
    else:
        srt, usrt = x, np.asarray(y)

    # for dist == 'default', generate the default distance function
    if dist == 'default':
        dist = lambda a,b: abs(b-a)

    # if D is specified, zero everything except the upper triangle
    if D is not None:
        D = np.triu(D, 1)

    # if dist is specified, populate D
    if D is None and dist is not None:
        D = _distmat(srt, assume_sorted=True, dist=dist)

    S = _swapmat(srt, usrt)
    return S, D

def _normal_kendall_tau(S):
    'Calculate the regular Kendall Tau'
    N = len(S)
    distance = np.sum(S)
    normalized_distance = distance * 2 / (N * (N-1))
    correlation_var = 2 * (2*N + 5) / (9*N * (N - 1))
    return distance, normalized_distance, correlation_var

def _general_kendall_tau(S, D):
    'Calculate the generalized Kendall Tau with pairwise weights D'
    N = len(S)
    distance = np.sum(S*D)
    Dsum = np.sum(D)
    normalized_distance = distance / Dsum
    correlation_var = -1 + 4 * _expected_QD2(D) / (Dsum**2)
    return distance, normalized_distance, correlation_var

def _expected_QD2(D):
    'Calculate E[Q_D**2] with Q_D the generalized Kendall distance'
    N = len(D)
    QD2 = 0
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
            QD2 += D[i,j] * D[k,L] * p
    return QD2

def _estimated_expected_QD2(D, nsamples=1000):
    N = len(D)
    ordered = np.arange(N)
    # chain of generators
    samples = (np.random.permutation(N) for _ in range(nsamples))
    Ss = (_swapmat(ordered, sample) for sample in samples)
    QDs = (np.sum(S*D) for S in Ss)
    QD2s = (QD**2 for QD in QDs)
    return sum(QD2s) / nsamples

def _zscore_to_pvalue(zscore, tail='two-sided'):
    '''Return the associated p-value from a given Z-score.
    @param tail from ('two', 'left', 'right')
    '''
    if tail == 'two':
        return 2 * scipy.stats.norm.sf(abs(zscore))
    elif tail == 'left':
        return 1 - scipy.stats.norm.sf(zscore)
    elif tail == 'right':
        return scipy.stats.norm.sf(zscore)
    else:
        raise ValueError(f'Unsupported tail {repr(tail)}')

def _swapmat(x, y=None):
    '''Return integer matrix of elements out of order (upper-triangular)

    If x and y are specified, then x is the true order and y is the unordered.
    If only x is specified, then x is unordered and will be compared with
    sorted(x).

    The normal Kendall tau distance is simply the sum of all entries.

    >>> _swapmat(['a', 'b', 'c'], ['b', 'c', 'a'])
    array([[0, 1, 1],
           [0, 0, 0],
           [0, 0, 0]])

    >>> _swapmat([5, 1, 2, 4, 3])
    array([[0, 0, 0, 0, 1],
           [0, 0, 0, 0, 1],
           [0, 0, 0, 1, 1],
           [0, 0, 0, 0, 1],
           [0, 0, 0, 0, 0]])
           
    '''
    x = np.asarray(x)
    if y is None:
        srt, usrt = x.copy(), x
        srt.sort()
    else:
        srt, usrt = x, np.asarray(y)
    N = len(x)
    v = np.zeros((N, N), dtype=int)
    idx = np.argsort(usrt) #[usrt.index(val) for val in srt]
    for j in range(1, N):
        for i in range(j):
            v[i, j] = int(idx[i] > idx[j])
    return v

def _distmat(x, assume_sorted=False, dist=None):
    '''Return a distance matrix between elements of x

    @param x: vector of numerical values
    @param assume_sorted: True means do not bother sorting x
    @param dist: distance function between two values of x

    >>> _distmat([1,3,6,10], assume_sorted=True)
    array([[0., 2., 5., 9.],
           [0., 0., 3., 7.],
           [0., 0., 0., 4.],
           [0., 0., 0., 0.]])
    '''
    x = np.asarray(x)
    if not assume_sorted:
        x = x.copy()
        x.sort()
    if dist is None:
        dist = lambda a, b: b - a
    N = len(x)
    v = np.zeros((N, N), dtype=float)
    for j in range(1, N):
        for i in range(j):
            v[i, j] = dist(x[i], x[j])
    return v
