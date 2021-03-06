eps = 0.01;
$fn = 120;

use <./helpers.scad>

rot_x(90)
  difference() {
    union() {
      main_body();
      tabs();
    }
    screw_holes();
  }



//
// modules
//

module main_body() {
  intersection() {
    difference() {
      linear_extrude(25.4, scale = [1, 18.56 / 3.2])
        mov_y(1.6)
        square([43.2, 3.2], center = true);
      mov_xyz(-43.2 / 2 + 3.2, -5, 13.2) cube(43.2 - 2*3.2);
    }
    union() {
      mirror([1, 0, 0]) mov_x(-5) tab(r = 5, h = 10);
      mov_x(-5) tab(r = 5, h = 10);
      mov_x(-25) cube([50, 11.7 - 2, 30]);
      mov_x(-25) cube([50, 30, 18.4 - 0.5]);
    }
  }
}

module tabs() {
  tab(r = 1.9, h = 5);
  mirror([1, 0, 0]) tab(r = 1.9, h = 5);
}

module screw_holes() {
  screw_hole();
  mirror([1, 0, 0]) screw_hole();
}

module tab(r, h) {
  translate([43.2/2 + 5/2 - 0.4, 11.7 - 2, 18.4 + 2])
    rot_y(90) cylinder(r = r, h = h, center = true);
}

module screw_hole() {
  screw_head_radius = 5.8 / 2;
  screw_head_clearance = 1;
  height = 12;
  screw_shaft_radius = 3.5 / 2;

  center = [43.2/2 - 7.5 - screw_shaft_radius, 0,
            6.5 + screw_shaft_radius];

  translate(center)
    mov_y(height/2 + 3.2)
    rot_x(90)
    cylinder(r = screw_head_radius + screw_head_clearance / 2,
             h = height, center = true);

  translate(center)
    rot_x(90)
    cylinder(r = screw_shaft_radius, h = height, center = true);
}
