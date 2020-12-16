/* [Print Settings] */

// Line width being used for the slice
line_width = 0.04;

// Layer height being used for the slice
layer_height = 0.01;


/* [Model Settings] */

// Which model to render
part = "nothing"; // ["nothing", "ball", "socket", "all"]

// magnet cube size
magnet_size = 1.0;

// screw radius
screw_radius = 1.59;

// piece radius, the actual printed piece
piece_radius = 0.8;

// ball radius
ball = 0.55;

// rod radius
rod_radius = 0.234;

// length of the rod from the joint socket
rod_length = 1;

/* [Advanced Settings] */

// clearance for fitting together (e.g., for the socket joint)
clearance = 0.13;

// number of faces for spheres and cylinders
num_curved_faces = 100;



use <./helpers.scad>

$fn = num_curved_faces;

if (part == "ball") {
  ball();
} else if (part == "socket") {
  socket();
} else if (part == "all") {
  //intersection() {
    union() {
      ball();
      socket();
      //#magnet();
      //#screw();
    }
  //  mov_xz(-10, -10) cube(20);
  //}
}

module ball(grow_by = 0) {
  a = rod_radius;
  b = ball;
  c = sqrt(b*b - a*a);
  union() {
    mov_z(magnet_size + b - c + clearance/2) sphere(b + grow_by);
    mov_z(magnet_size + b) cylinder(r=a, h=rod_length + rod_radius);
  }
}

module socket() {
  size = magnet_size;
  difference() {
    mov_z(-magnet_size + clearance)
      cylinder(r=piece_radius, h=magnet_size*2 + ball - clearance);
    magnet(clearance);
    ball(clearance);
  }
  // base-support
  mov_z(-magnet_size + clearance)
    linear_extrude(layer_height) {
      intersection() {
        union() {
          square([magnet_size * 2, magnet_size / 3], center=true);
          square([magnet_size / 3, magnet_size * 2], center=true);
        }
        circle(piece_radius);
      }
    }
}

module magnet(grow_by = 0) {
  size = magnet_size + grow_by;
  mov_z(-magnet_size/2) cube(size, center=true);
}

module screw() {
  r = screw_radius;
  mov_z(-magnet_size) rot_x(180) linear_extrude(height=r*5, scale=0) circle(r);
}
