/* [Print Settings] */

// Line width being used for the slice
line_width = 0.45;

// Layer height being used for the slice
layer_height = 0.2;

clearance = 0.2;

eps = 0.04;
$fn = 200;


/* [Model Settings] */

type             = "L"; // ["R", "L"]
frame_height     = 19.1;
frame_width      = 19.5;
thickness        = 3.8; // 2.8
hole_diameter    = 6.7;
orig_base_length = 72.5;


/* [Joint Size] */

// joint is from the screw center at the bottom (before the brace) to the
// center of the fastning hole to the wall
joint_x_dist = -frame_width / 2 - clearance * 2;
joint_y_dist = 106.2; // not too precise
joint_z_dist =  86.5; // was 90.1


/* [Externals] */

show_external = false;

fastener_length        = 33.8;
fastener_head_diameter = 12.8;
fastener_head_height   = 1.7;
fastener_body_diameter = 6.4;


joint = [joint_x_dist, joint_y_dist, joint_z_dist];
base_length = orig_base_length/2 + joint_y_dist + hole_diameter*2;
base_width  = frame_width + clearance + thickness;

use <./helpers.scad>

if (type == "L") {
  mirror([0, 1, 0]) right_piece();
} else {
  right_piece();
}

module right_piece() {
  if (show_external) {
    #crib_bar();
    #fastener_1();
    #fastener_2();
  }

  bottom_plate();
  side_plate();
  side_support();
  mov_y(-4 * hole_diameter + thickness/2) side_support();
}

module bottom_plate() {
  W = base_width;
  L = base_length;
  H = thickness;
  hole_L = hole_diameter*3;
  difference() {
    translate([-W + frame_width/2, -orig_base_length/2, -H-clearance])
      cube([W, L, H]);
    mov_z(-H-eps/2-clearance)
      linear_extrude(height=H+eps) {
        hull() {
          // 45 degrees instead of circles to make more printable
          mov_y(-hole_L/2) rot_z(45) square(hole_diameter/sqrt(2), center=true);
          mov_y( hole_L/2) rot_z(45) square(hole_diameter/sqrt(2), center=true);
          //translate([0, -hole_L/2]) circle(hole_diameter/2);
          //translate([0,  hole_L/2]) circle(hole_diameter/2);
        }
      }
      //mov_z(-H-eps/2-clearance) cylinder(h=H+eps, r=hole_diameter/2);
  }
}

module side_plate() {
  W = base_width;
  L = base_length;
  H = thickness;
  rot_y(-90)
    mov_z(frame_width/2+clearance)
    linear_extrude(H) {
      difference() {
        hull() {
          mov_xy(-H-clearance, -orig_base_length/2) square([W, L]);
          mov_xy(joint_z_dist, joint_y_dist) circle(hole_diameter*2);
        }
        mov_xy(joint_z_dist, joint_y_dist) circle(hole_diameter/2);
      }
    }
}

module side_support() {
  T = thickness;
  W = base_width;
  L = base_length;
  X = joint_x_dist;
  Y = L - orig_base_length/2;
  Z = joint_z_dist;
  //translate([X-T+clearance, Y-T, -clearance]) linear_extrude(Z) square([W, T]);
  hull() {
    translate([X-T+clearance, Y-T, -T-clearance])
      cube([W, T, T]);
    translate([X-T+clearance, Y-T/2, Z-T])
      cube([T, T/2, T]);
  }
}

module crib_bar() {
  L = orig_base_length + 20;
  H = frame_height;
  W = frame_width;
  translate([-W/2, -L/2, 0]) cube([W, L, H]);
}

module fastener_1() {
  body_h = fastener_length - fastener_head_height + eps;
  body_shift_z = -body_h + frame_height + eps;
  body_r = fastener_body_diameter / 2;
  head_h = fastener_head_height;
  head_shift_z = frame_height;
  head_r = fastener_head_diameter / 2;
  union() {
    mov_z(body_shift_z) cylinder(h=body_h, r=body_r);
    mov_z(head_shift_z) cylinder(h=head_h, r=head_r);
  }
}

module fastener_2() {
  body_h = fastener_length - fastener_head_height + eps;
  body_shift_z = -body_h + eps;
  body_r = fastener_body_diameter / 2;
  head_h = fastener_head_height;
  head_shift_z = body_h;
  head_r = fastener_head_diameter / 2;
  translate(joint)
    rot_y(90)
    union() {
      mov_z(body_shift_z) cylinder(h=body_h, r=body_r);
      cylinder(h=head_h, r=head_r);
    }
}

