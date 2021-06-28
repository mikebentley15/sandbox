/* [General Options] */

height        =   3;
// will adjust wrench size to match the nut for this type of screw
screw_size    =   4;
length        =  60;
width         =   8;
nut_clearance =   0.34;

// how big the circle around the hex cutout is compared to the nut diameter
outer_diameter_factor = 2.15;

$fn = 30;

use <screwlib.scad>
use <helpers.scad>


//
// actual implementation
//

wrench(length, width, screw_size, height, outer_diameter_factor);


//
// main functions
//

module wrench(L, w, size, h, diameter_factor = 2.15) {
  // figure out dimensions
  nut_radius = M_nut_outer_radius(size) + nut_clearance;
  outer_radius = diameter_factor * nut_radius;

  echo(nut_radius = nut_radius);

  // create a 2D image of what I want, then just linear extrude it

  linear_extrude(h)
  difference() {
    union() {
      // handle
      assert(L > w);
      stretch_x(L - w)
        circle(d = w);

      // head
      circle(outer_radius);
    }

    // cutouts
    union() {
      stretch_x(- outer_radius)
        mov_x(- nut_radius * diameter_factor * 0.40)
        circle(nut_radius, $fn = 6);
    }
  }
}


//
// helper functions
//

// stretch with the given displacement
module stretch(displacement)
  { hull() { children(); translate(displacement) children(); } }
module stretch_x(dx) { stretch([dx, 0, 0]) children(); }
module stretch_y(dy) { stretch([0, dy, 0]) children(); }
module stretch_z(dz) { stretch([0, 0, dz]) children(); }
