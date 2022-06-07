import numpy as np

def standardize(x, *, inplace=False):
    '''
    Return a copy of x with each column transformed to mean 0 and stdev 1.
    If you specify inplace=True, then modifies x in-place.
    '''
    if not inplace:
        x = x.copy()
    x = (x - x.mean(axis=1)[:, np.newaxis]) / x.std(axis=1)[:, np.newaxis]
    return x

def pca(x):
    '''
    Return the principal components of x as column vectors of a returned
    matrix.  Also returns the eigenvalues for each principal component.

    @return (eigenvectors, eigenvalues)
    '''
    x = standardize(x)
    cov = np.cov(x.T)
    eigvals, eigvecs = np.linalg.eig(cov)
    return (eigvecs, eigvals)

def projection(x, y):
    '''
    Returns the projection of x onto y.
    '''
    return y * (x.dot(y) / y.dot(y))

def orthonormalization(x, *, inplace=False):
    '''
    Return a copy of x with each column being orthogonal to the previous ones.
    First column will be unchanged.

    This function does not check that the columns of x are not colinear, it is
    the responsability of the user to ensure this is the case before calling
    this function.

    If you specify inplace=True, then modifies x in-place.
    '''
    if not inplace:
        x = x.copy()
    ncols = x.shape[1]
    # Grahm-Schmidt Orthogonalization
    for i in range(1, ncols):
        x[:, i] -= projection(x[:, i], x[:, i-1])
    return x

def pca_bounding_box(x):
    '''
    Return a non-axis-aligned bounding box for a set of points x using PCA.

    @param x: NxM numpy matrix in R^M space (e.g., N points in R^3 for M=3)
    @return (corner, extents)
    - corner: a point in R^M denoting a corner of the bounding box
    - extents: an MxM matrix with column vectors describing the extents of the
      box, in PCA order, after orthogonalization, i.e., the PCA eigenvectors
      times each length.
    '''
    M = x.shape[1]
    eigvecs, _ = pca(x)

    # PCA already has them orthogonal, but not orthonormal
    #R = orthonormalization(eigvecs, inplace=True)
    R = eigvecs
    R /= np.linalg.norm(R, axis=1)[:, np.newaxis]

    # rotate into the orthonormal basis and find the box
    x_rot = x.dot(R)
    min_corner_rot = x_rot.min(axis=1)
    max_corner_rot = x_rot.max(axis=1)
    extents_rot = max_corner_rot - min_corner_rot

    # rotate back before returning
    return R.dot(min_corner_rot), R.dot(np.ones((M, M)) * extents_rot)
