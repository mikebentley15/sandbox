from collections import namedtuple

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
    '''
    N = len(x)

    # populate srt and usrt for sorted and unsorted arrays, respectively
    if y is None:
        srt, usrt = x.copy(), x
        srt.sort()
    else:
        srt, usrt = x, y

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
    #print(f'srt:  {srt}')
    #print(f'usrt: {usrt}')
    #print(f'S:\n{S}')
    #print(f'D:\n{D}')
    if D is None:
        return _normal_kendall_tau(S)
    else:
        return _general_kendall_tau(S, D)

def _normal_kendall_tau(S):
    N = len(S)
    distance = np.sum(S)
    normalized_distance = distance * 2 / (N * (N-1))
    correlation = 1 - 2 * normalized_distance
    correlation_var = 2 * (2*N + 5) / (9*N * (N - 1))
    zscore = correlation / np.sqrt(correlation_var)
    pvalue = _zscore_to_pvalue(zscore)
    return KendallTau(distance, normalized_distance, correlation, zscore, pvalue)

def _general_kendall_tau(S, D):
    N = len(S)
    distance = np.sum(S*D)
    Dsum = np.sum(D)
    normalized_distance = distance / Dsum
    correlation = 1 - 2 * normalized_distance
    correlation_var = -1 + 4 * _expected_QD2(D) / (Dsum**2)
    zscore = correlation / np.sqrt(correlation_var)
    pvalue = _zscore_to_pvalue(zscore)
    return KendallTau(distance, normalized_distance, correlation, zscore, pvalue)

def _expected_QD2(D):
    N = len(D)
    QD2 = 0
    for j in range(N):
        for i in range(j):
            for k in range(N):
                for L in range(k):
                    if i == k and j == L:
                        p = 0.5
                    elif i != k and i != L and j != k and j != L:
                        p = 0.25
                    else:
                        p = 1./3.
                    QD2 += D[i,j] * D[k,L] * p
    return QD2

def _zscore_to_pvalue(zscore):
    return scipy.stats.norm.sf(zscore)

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
    '''
    if y is None:
        srt, usrt = x.copy(), x
        srt.sort()
    else:
        srt, usrt = x, y
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

def _inversion_vector(x, S=None):
    if S is None:
        S = swapmat(x)
    return S.sum(axis=0)

def _inversion_vector_dist(x, S=None, D=None):
    return (S*D).sum(axis=0)

def _tau_dist(vec, S=None, D=None):
    if S is None:
        S = swapmat(x)
    if D is None:
        D = distmat(x)
    return np.sum(S*D)

def _tau_dist_normalized(vec, S=None, D=None):
    if D is None:
        D = distmat(vec)
    return tau_dist(vec, S=S, D=D) / D.sum()

