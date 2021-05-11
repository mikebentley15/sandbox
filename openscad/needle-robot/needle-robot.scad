/* [Platform] */

platform_width  = 60;
platform_height = 32;
platform_depth  = 42;

platform_side_screw_side_distance = 10;
platform_side_screw_top_distance  = 6;

platform_top_screw_width_side_distance = 15;
platform_top_screw_depth_side_distance = 6;

platform_screw_size = 4;


/* [L Bracket] */

L_bracket_height    = 98;
L_bracket_length    = 120;
L_bracket_width     = 24;
L_bracket_thickness = 2.9;

L_bracket_slit_width         = 5.2;
L_bracket_slit_end_distance  = 12;
L_bracket_bottom_slit_length = 90;
L_bracket_side_slit_length   = 70;




/* [Model Settings] */

// Which model to render
part = "nothing"; // ["nothing", "logo", "helix"]

// Logo sphere radius
sphere_r = 50;

// Helix turns
turns = 2;

// Extra length when making holes
eps = 0.04;

/* [Curve Settings] */

// How long the sections of a curved surface should be
segment = 10;

use <./helpers.scad>
use <screwlib.scad>

$fn = 20;

if (part=="logo") { logo(sphere_r, segment); }
module logo(sphere_r, segment)
{
  module cut_cylinder(sphere_r)
  {
    cylinder_r = sphere_r/2;
    cylinder_fn = get_fn_r(cylinder_r, s=segment);

    rot_z(180/cylinder_fn)
      cylinder(r=cylinder_r,
               h=2*eps + 2*sphere_r,
               $fn=cylinder_fn,
               center=true);
  }

  sphere_circ = PI*2*sphere_r;
  sphere_fn = 8 * round(sphere_circ/segment);

  difference() {
    fibonacci_sphere(sphere_r, sphere_fn);

    cut_cylinder(sphere_r);

    rot_y(90)
      cut_cylinder(sphere_r);

    rot_x(90)
      cut_cylinder(sphere_r);
  }
}




if (part=="helix") { helix(sphere_r, segment); }
module helix(outer_r, segment)
{
  inner_r = 0.5 * outer_r;
  inner_fn = get_fn_r(inner_r, s=segment);
  height = 3*outer_r;
  slices = turns * round(height/segment);

  linear_extrude(height=3*outer_r,
                 twist=360*turns,
                 slices=slices,
                 center=true)
    mov_x(outer_r - inner_r)
    circle(r=inner_r, $fn=inner_fn);
}
