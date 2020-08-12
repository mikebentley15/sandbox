/* [Dimensions] */

// millimeters
thickness = 1.0;

// page width (inches)
width = 4.25;

// page length (inches)
length = 5.5;

// buffer around each edge to make cover bigger (inches)
cover_increase = 0.25;


/* [Message] */

enable_message = true;

message = "3D Printing Log";

font_size = 10;

// vertical offset of words (mm)
vertical_offset = 15;


/* [Holes] */

// millimeters
radius = 1.0;

// number of holes
n_holes = 2;

// distance from left (mm)
distance_in = 12;

// distance to the first hole from the top and bottom (mm)
distance_top = 25;


/* [Miscellaneous] */

// Extra length when making holes (mm)
eps = 0.01;

// How many edges for cylinder curves
cylinder_faces = 32;

mm_per_inch = 25.4;


/*** End of settings ***/

width_mm = mm_per_inch * (width + cover_increase * 2);
length_mm = mm_per_inch * (length + cover_increase * 2);


use <./helpers.scad>

module book_cover() {
  mov_z(thickness / 2)
    cube([width_mm, length_mm, thickness], center=true);
}

module book_holes() {
  dl = (length_mm - distance_top * 2) / (n_holes - 1);
  for (i = [0:n_holes - 1]) {
    mov_xyz(-width_mm / 2 + distance_in,
            -length_mm / 2 + distance_top + dl * i,
            thickness / 2)
      cylinder(h=thickness + eps,
               r=radius,
               center=true,
               $fn=cylinder_faces);
  }
}

difference() {
  book_cover();
  book_holes();
  if (enable_message) {
    mov_z(-eps)
      mov_y(vertical_offset)
      linear_extrude(thickness + 2 * eps)
      text(message, halign="center", size=font_size);
  }
}
