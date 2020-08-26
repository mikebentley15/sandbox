use <./helpers.scad>

$fn = 120;

rot_x(90) cylinder(h = 35, r = 2.5, center = true);

body();

tab_with_hole();

translate([28.2, 0, -0.9])
  rot_y(180)
  rot_x(90)
  mov_z(-6)
  fastener();

mov_x(11.6)
  rot_y(5)
  handle();


// Modules

module body() {
  minkowski() {
    mov_x(12.5) cube([24, 10, 2.2], center = true);
    sphere(r = 1.0);
  }
}

module tab_with_hole() {
  rot_y(-39)
    mov_z(-1.25)
    linear_extrude(2.5) {
      difference () {
        minkowski() {
          translate([-9.5, -2, 0]) square([7, 4]);
          circle(r = 3);
        }
        // hole
        mov_x(-8) circle(r = 1.5);
      }
    }
}

module fastener() {
  hull()
  {
    rotate_extrude() {
      union() {
        square([2, 12]);
        mov_xy(2, 1) circle(1);
        mov_xy(2, 11) circle(1);
        mov_y(1) square([3, 10]);
      }
    }
    translate([4, -2,  1]) rot_y(90) cylinder(r = 1, h = 0.2);
    translate([4, -2, 11]) rot_y(90) cylinder(r = 1, h = 0.2);
    translate([4,  5,  1]) rot_y(90) cylinder(r = 1, h = 0.2);
    translate([4,  5, 11]) rot_y(90) cylinder(r = 1, h = 0.2);
  }
}

module handle() {
  //mov_y (-20)
  minkowski() {
    mov_xy(1, -5) cube([2, 10, 23]);
    sphere(1);
  }

  // 8 x 39 x 15.5 moved up to have the top at 28.5
  difference() {
    intersection() {
      hull() {
        mov_xyz(4, -15.5, 24.5) sphere(4);
        mov_xyz(4,  15.5, 24.5) sphere(4);
        mov_xyz(4, -15.5, 12.5) sphere(4);
        mov_xyz(4,  15.5, 12.5) sphere(4);
      }
      minkowski() {
        mov_xyz(2, -17.5, 15.5) cube([2, 35, 11]);
        sphere(2);
      }
    }
    hull() {
      mov_xyz(7, -14.5, 23.5) sphere(3);
      mov_xyz(7,  14.5, 23.5) sphere(3);
      mov_xyz(7, -14.5, 10.0) sphere(3);
      mov_xyz(7,  14.5, 10.0) sphere(3);
    }
  }
}
