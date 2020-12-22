/* [Print Settings] */

// Line width being used for the slice
line_width = 0.45;

// Layer height being used for the slice
layer_height = 0.2;


/* [Cookie Settings] */

// Scale
svg_scale = 0.555;

// Outline of the cookie cutter
outline_file = "outline.svg";

// height of the cookie outline piece
outline_height = 20.0;

// use the decals_file
use_decals = true;

// Decals of the cookie cutter
decals_file = "decals.svg";

// height of the decals piece (shorter than outline height)
decals_height = 15.0;

// use the backing_file
use_backing = true;

// filled in shape of the backing for rigidity
backing_file = "filled.svg";

// height of the backing section
backing_height = 1.0;


eps = 0.04;

use <./helpers.scad>

//mov_z(-5) square(mm(3));

union() {

  linear_extrude(outline_height)   svg(outline_file);

  if (use_decals) {
    linear_extrude(decals_height)  svg(decals_file);
  }

  if (use_backing) {
    difference() {
      linear_extrude(backing_height) svg(backing_file);
      scale([svg_scale, svg_scale, 1.0])
        translate([90, 60, -1]) cylinder(r=18, h=10, $fn=200);
    }
  }

}

module svg(filename) {
  scale([svg_scale, svg_scale, 1.0]) import(filename);
}

//union() {
//  linear_extrude(backing_height)
//  union() {
//    import(backing_file);
//    import(decals_file);
//    import(outline_file);
//  }
//
//  mov_z(backing_height - eps)
//  linear_extrude(decals_height - backing_height + eps)
//  union() {
//    import(decals_file);
//    import(outline_file);
//  }
//
//  mov_z(decals_height - eps)
//    linear_extrude(outline_height - decals_height + eps)
//    import(outline_file);
//}
