#!/usr/bin/env python

import sys

import numpy as np
import trimesh

def create_meshes():
    meshes = []

    # mesh objects can be created from existing faces and vertex data
    meshes.append(trimesh.Trimesh(vertices=[[0, 0, 0], [0, 0, 1], [0, 1, 0]],
                                  faces=[[0, 1, 2]]))

    # by default, Trimesh will do light processing, which will remove any NaN
    # values and merge vertices that share the same position.  If you do not
    # want to do this on load, you can pass `process=False`
    meshes.append(trimesh.Trimesh(vertices=[[0, 0, 0], [0, 0, 1], [0, 1, 0]],
                                  faces=[[0, 1, 2]],
                                  process=False))

    # mesh objects can be loaded from a file name or from a buffer.  You can
    # pass any of the kwargs for the `Trimesh` constructor to `trimesh.load`,
    # including `process=False` if you would like to preserve the original
    # loaded data without merging vertices.  However, STL files will be a soup
    # of disconnected triangles without merging vertices and will not register
    # as watertight.
    meshes.append(trimesh.load('models/featuretype.stl'))

    return meshes

def analyze_mesh(mesh):
    print('is watertight:                ', mesh.is_watertight)
    print('euler_number:                 ', mesh.euler_number)
    print('volume:                       ', mesh.volume)
    if mesh.is_watertight:
        # the convex hull is another trimesh object
        print('convex hull volume:           ', mesh.convex_hull.volume)
        print('vol. / conv. hull vol.:       ', mesh.volume / mesh.convex_hull.volume)
        print('center of mass:               ', mesh.center_mass)
        print('inertia:                      ', mesh.moment_inertia)
        print('axis-aligned bounding box:    ', mesh.bounding_box.extents)
        print('oriented bounding box:        ', mesh.bounding_box_oriented.primitive.extents)
        print('orientation of bounding box:  ', mesh.bounding_box_oriented.primitive.transform)
        print('oriented bounding box volume: ', mesh.bounding_box_oriented.volume)
        print('bounding cylinder volume:     ', mesh.bounding_cylinder.volume)
        print('bounding sphere volume:       ', mesh.bounding_sphere.volume)


def split_mesh(mesh):
    splits = mesh.split()

    # center each mesh on their center of mass
    for split in splits:
        split.vertices -= split.center_mass

    return splits

def main(arguments):
    # attach to logger so trimesh messages will be printed to console
    #trimesh.util.attach_to_log()
    meshes = create_meshes()
    for mesh in meshes:
        mesh.vertices -= mesh.center_mass
        analyze_mesh(mesh)
        pieces = split_mesh(mesh)
        try:
            mesh.show()
        except:
            pass
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
