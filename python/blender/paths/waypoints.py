# blender [<from-blend-file>] --background --python waypoints.py -- <points-csv>

# Example code came from
#   https://behreajj.medium.com/scripting-curves-in-blender-with-python-c487097efd13

import bpy
from bpy import ops
from mathutils import Vector

import argparse
import csv
import os
import sys

workdir = os.path.dirname(os.path.abspath(__file__))
#sys.path.append(workdir)

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'Creates a blend file with the given waypoints as a path'
    parser.add_argument('problem_toml')
    parser.add_argument('configs_csv')
    return parser

def main(arguments):
    print(sys.version)
    print(sys.executable)

    sys.path.append(os.path.join(
        os.environ['HOME'], 'git', 'armlab', 'tendon_experiments', 'build-blender'))

    import cpptendon as tend

    parser = populate_parser()
    args = parser.parse_args(arguments)

    prob = tend.motion_planning.Problem.from_toml(args.problem_toml)
    qs = prob.load_plan(args.configs_csv)
    shapes = [prob.robot.forward_kinematics(q) for q in qs]

    for i, shape in enumerate(shapes):
        ops.curve.primitive_bezier_curve_add(enter_editmode=True)
        ops.curve.subdivide(number_cuts=len(shape)-2)

        curve = bpy.context.active_object
        curve.name = f'backbone_{i:03d}'
        spline = curve.data.splines[-1]
        bezier_points = spline.bezier_points
        for knot, pt in zip(bezier_points, shape):
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

    print('Creating waypoints.blend')
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(workdir, 'waypoints.blend'))

if __name__ == '__main__':
    if '--' in sys.argv:
        main(sys.argv[sys.argv.index('--') + 1 : ])
