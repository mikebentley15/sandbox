/* [Bath Bomb Settings] */

part = "all"; // ["all", "top", "bottom", "shell"]

// shape to use for mold
egg_stl = "Dragon_Egg_Mi_solid-fixed-18k.stl";

// size of the egg_stl (roughly)
egg_size    = [160, 210, 160];

// size of the dragon inside
dragon_size = [45, 64, 29];

// percent of how much bath bomb to have around dragon.  This determines the scaling of the stl file.
dragon_padding_percent = 50;

// percent around mold for containner to be
mold_padding_percent = 20;

shell_width = 6;
base_height = 6;
shell_clearance = 0.4;

include_handle = false;


/* [Print Settings] */

// Line width being used for the slice
line_width = 0.45;

// Layer height being used for the slice
layer_height = 0.2;


/* [Model Settings] */

// Extra length when making holes
eps = 0.04;

// number of faces for circles
$fn = 100;


use <./helpers.scad>

dragon_pad = dragon_size * (1 + dragon_padding_percent / 100);
stl_scale = max([for (i = [0:2]) dragon_pad[i] / egg_size[i]]);
scaled_egg_size = egg_size * stl_scale;
body_scale = (1 + mold_padding_percent/100);
body_size = scaled_egg_size * body_scale;

if      (part == "all")    { full_model(); shell(); }
else if (part == "shell")  { shell(); }
else if (part == "top")    { top(); }
else if (part == "bottom") { bottom(); }

module full_model() {
  difference() {
    union() {
      // elliptical cylinder main body
      scale(body_size) cylinder(d=1, h=1, center=true);

      // handle on top
      if (include_handle) {
        mov_z(body_size[2] / 2 - eps)
          linear_extrude(20 + eps, scale=0.5) circle(d=30);
        mov_z(body_size[2] / 2 + 25) sphere(d=30);
      }

      // base on bottom
      mov_z(-body_size[2] / 2 - eps)
        scale([
            body_size[0] + shell_clearance + shell_width,
            body_size[1] + shell_clearance + shell_width,
            base_height
          ])
        cylinder(d=1, h=1);
    }
    scale(stl_scale) rot_x(90) import(egg_stl);
  }
}

module shell() {
  mov_z(base_height/2 + shell_clearance/2 + eps)
    difference() {
      scale([
          body_size[0] + shell_clearance + shell_width,
          body_size[1] + shell_clearance + shell_width,
          body_size[2] - base_height - shell_clearance
        ])
        cylinder(d=1, h=1, center=true);
      scale(body_size + (shell_clearance + eps) * [1,1,1])
        cylinder(d=1, h=1, center=true);
    }
}

module top() {
  intersection() {
    full_model();
    translate([-body_size[0], -body_size[1], 0]) cube(4 * body_size);
  }
}

module bottom() {
  difference() {
    full_model();
    translate([-body_size[0], -body_size[1], 0]) cube(4 * body_size);
  }
}

