/* [Model Settings] */

base_height = 2.0;

foot_radius = 11.5;

wall_height = 3.0;

wall_thickness = 3.0;


/* [Advanced Settings] */

// Extra length when making holes
eps = 0.04;

circle_faces = 200;


$fn = circle_faces;

use <./helpers.scad>


// outer cylinder
r_o = foot_radius + wall_thickness;
h_o = base_height + wall_height;

// inner cylinder
r_i = foot_radius;
h_i = wall_height + eps;

difference() {
  cylinder(r=r_o, h=h_o);
  mov_z(base_height)
    cylinder(r=r_i, h=h_i);
}
