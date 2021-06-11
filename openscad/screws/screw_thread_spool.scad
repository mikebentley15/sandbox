/* [Pieces] */

piece = "nut"; // ["all", "screw", "nut"]

/* [Screw Parameters] */

head_diameter = 6.56;
head_height   = 4;
size          = 4;
pitch         = 1.25;
smooth_height = 5.5;
// spool section for winding thread
spool_height  = 3;
spool_flare   = 0.85;
thread_height = 10;
hole_diameter = 0.8;
hole_z        = 1;
allen_wrench_inner_diameter = 2.7;
allen_wrench_depth    = 3;
nut_length    = 4;
nut_clearance = 0.5;

allen_wrench_diameter = allen_wrench_inner_diameter / cos(30);

eps = 0.01;
$fn = 50;

use <screwlib.scad>

if (piece == "screw") {
  my_screw();
}
if (piece == "nut") {
  my_nut();
}
if (piece == "all") {
  my_screw();
  translate([15, 0, 0]) my_nut();
}

module my_screw() {
  difference() {
    union() {
      translate([0, 0, spool_height + smooth_height])
        screw_shaft(size, thread_height, pitch, profile=2);
      translate([0, 0, spool_height - eps])
        cylinder(d=size, h=smooth_height + 2*eps);
      translate([0, 0, -eps])
        cylinder(r1=size/2, r2=size/2 + spool_flare, h=spool_height+eps);
      translate([0, 0, -head_height])
        cylinder(d=head_diameter, h=head_height, $fn=6);
    }
    translate([0, 0, hole_diameter/2 + hole_z])
      rotate([90, 0, 0])
      cylinder(d=hole_diameter, h=head_diameter+1.5, center=true);
    translate([0, 0, -head_height - eps])
      cylinder(d=allen_wrench_diameter, h=allen_wrench_depth+eps, $fn=6);
  }
}

module my_nut() {
  nut(size + nut_clearance,
      M_nut_inner_diameter(size),
      pitch,
      nut_length,
      profile=3);
}
