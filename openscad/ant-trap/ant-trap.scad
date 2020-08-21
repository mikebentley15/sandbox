/* [General Shape] */

// values are "top" or "bottom"
part = "top";

thickness = 4.0;

height = 15.0;

radius = 40.0;

rim = 10.0;

// angle up for the roof (90 degrees is vertical
pitch = 55.0;


/* [Slits] */

// number of vertical slits
n_slits = 12;

// width of slit hole
slit_width = 4.0;

// slit height reduction (usually from thickness on bottom to thickness on top)
slit_height_reduction = 0.4;


/* [Model Settings] */

// Extra length when making holes (mm)
eps = 0.01;

// How many edges for cylinder curves
cylinder_faces = 300;

// How much clearance to give for stackability
clearance = 0.3;



/*** End of settings ***/


use <./helpers.scad>

module top_general_shape(clearance = 0.0) {
  top_radius = radius - (height / tan(pitch));
  union() {
    mov_z(thickness - eps)
      cylinder(r1 = radius + clearance,
               r2 = top_radius + clearance,
               h = height + eps,
               $fn = cylinder_faces);
    cylinder(r = radius + rim,
             h = thickness,
             $fn = cylinder_faces);
  }
}

module slits(n, r, w, h) {
  dtheta = 360.0 / n;
  for (i = [1:n]) {
    rot_z(dtheta * i)
      mov_xy(-r, -w/2)
      cube([r, w, h]);
  }
}

module bottom_general_shape(clearance = 0.0) {
  pitch_addition = thickness / tan(pitch);
  intersection() {
    cylinder(r = radius + rim + eps + clearance,
             h = thickness * 2,
             $fn = cylinder_faces);
    cylinder(r1 = radius + rim - pitch_addition + eps + clearance,
             r2 = radius + rim + pitch_addition + eps + clearance,
             h = thickness * 2,
             $fn = cylinder_faces);
  }
}

module bottom_shape(clearance = 0.0) {
  difference() {
    bottom_general_shape();
    mov_z(thickness + eps)
      bottom_general_shape(clearance = clearance);
  }
}

module top_shape() {
  difference() {
    top_general_shape();
    mov_z(thickness + eps)
      slits(n = n_slits,
            r = radius,
            w = slit_width,
            h = height - thickness - 2*eps - slit_height_reduction);
    mov_z(-thickness - eps)
      top_general_shape(clearance = clearance);
    mov_z(-thickness + eps)
      bottom_shape(clearance = -clearance);
  }
}

if (part == "top") {
  top_shape();
} else {
  bottom_shape(clearance = clearance);
}
