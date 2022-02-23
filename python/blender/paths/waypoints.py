#!/usr/bin/env python3

# blender [<from-blend-file>] --background --python waypoints.py -- <points-csv>

# Example code came from
#   https://behreajj.medium.com/scripting-curves-in-blender-with-python-c487097efd13

try:
    import bpy
    from bpy import ops
    from mathutils import Vector
except ModuleNotFoundError:
    bpy = None

import argparse
import csv
import os
import shutil
import sys

workdir = os.path.dirname(os.path.abspath(__file__))
#sys.path.append(workdir)

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = f'''
        Creates a blend file with the given waypoints as a path.
        This script can be called directly, in which case it will forward the call to blender like so:
            blender --background
                [<blend-file>]
                [<blender-args> ...]
                --python {__file__}
                --
                <script-args>
        '''
    parser.add_argument('points_csv')
    parser.add_argument('-s', '--scale', type=float, default=1.0,
            help='scale points before making curves')
    parser.add_argument('-r', '--radius', type=float,
            help='radius of lines after putting on skin')
    parser.add_argument('-o', '--output', default='waypoints.blend',
            help='output blender file')

    # if called directly, add blender args only for --help to work nicely
    if bpy is None:
        populate_blender_args(parser)

    return parser

def populate_blender_args(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
        parser.description = 'Arguments for forwarding the call to blender'
    parser.add_argument('--blender-exe', default=shutil.which('blender'),
            help='blender executable to use')
    parser.add_argument('--blender-args',
            help='args to pass to blender as a comma-separated list')
    parser.add_argument('-b', '--blend-file',
            help='base blend file to add waypoints')
    return parser

def grouper(iterable, n):
    '''Groups a flat iterable into an iterable of n-tuples
    >>> list(grouper([1, 2, 3, 4, 5, 6, 7, 8], 2))
    [(1, 2), (3, 4), (5, 6), (7, 8)]
    >>> list(grouper([1, 2, 3, 4, 5, 6, 7, 8, 9], 3))
    [(1, 2, 3), (4, 5, 6), (7, 8, 9)]
    '''
    return zip(*[iter(iterable)]*n, strict=True)

def call_through_blender(arguments):
    import subprocess as subp
    parser = populate_blender_args()
    args, remaining_arguments = parser.parse_known_args(arguments)

    command = [args.blender_exe, '--background']
    if args.blend_file:     command.append(args.blend_file)
    if args.blender_args:   command.extend(args.blender_args.split(','))
    command.extend(['--python', __file__, '--'])
    command.extend(remaining_arguments)
    subp.call(command)

def main(arguments):
    print(sys.version)
    print(sys.executable)

    parser = populate_parser()
    args = parser.parse_args(arguments)

    if bpy is None:
        call_through_blender(arguments)
        sys.exit(0)

    with open(args.points_csv, 'r') as fin:
        reader = csv.reader(fin)
        shapes = [[Vector(pt) for pt in
                   grouper((args.scale * float(x) for x in row), 3)]
                  for row in reader]

        d = 1.5 * max((a - b).magnitude for shape in shapes
                      for a, b in zip(shape[:-1], shape[1:]))

        for pts in shapes:
            ops.curve.primitive_bezier_curve_add(enter_editmode=True)
            ops.curve.subdivide(number_cuts=len(pts)-2)

            curve = bpy.context.active_object
            spline = curve.data.splines[-1]
            bezier_points = spline.bezier_points
            for knot, pt in zip(bezier_points, pts):
                knot.co = pt
                # handle types:
                # - 'AUTO': smooth
                # - 'VECTOR': piecewise linear
                # - 'FREE': completely free
                knot.handle_left_type  = 'AUTO'
                knot.handle_right_type = 'AUTO'
                #knot.handle_left_type  = 'VECTOR'
                #knot.handle_right_type = 'VECTOR'

            ops.object.mode_set(mode='OBJECT')

            if args.radius:
                depth = d / 50
                ops.mesh.primitive_cylinder_add(
                    radius=args.radius,
                    depth=depth,
                    end_fill_type='NOTHING',#'TRIFAN',
                    vertices=32,
                )
                cylinder = bpy.context.active_object
                bpy.ops.object.shade_smooth()
                array_mod = cylinder.modifiers.new(name='Array', type='ARRAY')
                curve_mod = cylinder.modifiers.new(name='Curve', type='CURVE')

                array_mod.fit_type = 'FIT_CURVE'
                array_mod.curve = curve
                array_mod.use_relative_offset = True
                array_mod.relative_offset_displace = (0.0, 0.0, 1.0)
                array_mod.merge_threshold = depth / 3000
                array_mod.use_merge_vertices = True
                #array_mod.use_merge_vertices_cap = True

                curve_mod.object = curve
                curve_mod.deform_axis = 'POS_Z'

                ## Convert from a curve to a mesh.
                #ops.object.convert(target='MESH')
                #skin_mod = curve.modifiers.new(name='Skin', type='SKIN')
                #subsurf_mod = curve.modifiers.new(name='Subsurf', type='SUBSURF')

                ## Adjust modifier options.
                #skin_mod.use_smooth_shade = True
                #subsurf_mod.levels = 3

    print('Creating waypoints.blend')
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(workdir, args.output))

if __name__ == '__main__':
    if bpy:
        if '--' in sys.argv:
            main(sys.argv[sys.argv.index('--') + 1 : ])
        else:
            main([])
    else:
        main(sys.argv[1:])
