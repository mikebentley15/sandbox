/* [Print Settings] */

// Line width being used for the slice
line_width = 0.04;

// Layer height being used for the slice
layer_height = 0.02;


/* [Model Settings] */

// Which model to render
part = "nothing"; // ["nothing", "ball", "socket", "all"]

// magnet cube size
magnet_size = 1.0;

// screw radius
screw_radius = 0.7;

// ball radius
ball = 0.45;

// rod radius
rod_radius = 0.2;

// length of the rod from the joint socket
rod_length = 3;

/* [Advanced Settings] */

// clearance for fitting together (e.g., for the socket joint)
clearance = 0.08;

// Extra length when making holes
eps = 0.002;

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
  //  mov_z(-10) cube(20);
  //}
}

module ball(grow_by = 0) {
  a = rod_radius;
  b = ball;
  c = sqrt(b*b - a*a);
  union() {
    mov_z(magnet_size + 2*b - c + clearance) sphere(b + grow_by);
    mov_z(magnet_size + b) cylinder(r=a, h=rod_length);
  }
}

module socket() {
  size = magnet_size;
  difference() {
    mov_z(-magnet_size + clearance)
      cylinder(r=screw_radius, h=magnet_size*2 + ball*2 - clearance);
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
        circle(screw_radius);
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
