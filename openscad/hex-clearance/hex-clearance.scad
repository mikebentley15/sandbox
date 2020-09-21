/* [Print Settings] */

// Line width being used for the slice
line_width = 0.45;

// Layer height being used for the slice
layer_height = 0.2;


/* [Model Settings] */

// Hex rod radius
radius = 5;

// Hex rod length
length = 50;

n_holes = 9;
hole_clearance_start = 0.0;
hole_clearance_stop = 0.4;
sphere_joint_radius = 10;
hole_separation = 5;


/* [Model Settings] */

// Which model to render
part = "nothing"; // ["nothing", "rod", "holes"]

// Extra length when making holes
eps = 0.04;

fn = 50;

use <./helpers.scad>

if (part == "rod") { rod(); }
else if (part == "holes") { holes(); }

module rod() {
  rot_x(90) cylinder(r=radius, h=length, $fn=6, center=true);
}

module holes() {
  difference() {
    cx = (2*line_width + sphere_joint_radius) * 2;
    cy = 4*line_width + (n_holes * (sphere_joint_radius*2 + hole_separation))
         - hole_separation;
    cz = sphere_joint_radius;
    mov_y(cy/2 - sphere_joint_radius - 2*line_width)
      cube([cx, cy, cz], center=true);
    #spherical_shell_cutouts();
    #hex_cutouts();
  }
}

module spherical_shell_cutouts() {
  $fn = fn;
  d_clearance = (hole_clearance_stop - hole_clearance_start) / (n_holes-1);
  for (i = [1:n_holes]) {
    clearance = hole_clearance_start + (i-1) * d_clearance;
    mov_y((i-1) * (sphere_joint_radius * 2 + hole_separation))
      sphere_shell(sphere_joint_radius, sphere_joint_radius + clearance);
  }
}

module hex_cutouts() {
  d_clearance = (hole_clearance_stop - hole_clearance_start) / (n_holes-1);
  for (i = [1:n_holes]) {
    clearance = hole_clearance_start + (i-1) * d_clearance;
    echo("clearance (", i, ") = ", clearance);
    mov_y((i-1) * (sphere_joint_radius * 2 + hole_separation))
      cylinder(r=radius + clearance, h=2*sphere_joint_radius, center=true, $fn=6);
  }
}
