/* [Print Settings] */

// Line width being used for the slice
line_width = 0.45;

// Layer height being used for the slice
layer_height = 0.2;


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
