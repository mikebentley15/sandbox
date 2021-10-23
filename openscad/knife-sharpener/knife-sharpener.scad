/* [Print Settings] */

// Line width being used for the slice
line_width = 0.45;

// Layer height being used for the slice
layer_height = 0.2;

show_key = true;


/* [Model Settings] */

// Which model to render
part = "all"; // ["all", "handle", "base", "wall", "stand", "inserts"]

// Extra length when making holes
eps = 0.04;

// How many segments for curved surfaces
$fn = 20;


/* [Handle Settings] */

/* [Base Settings] */

// width of the base piece
base_width = 100.0;

// height of the base near the wall or stand
base_height_proximal = 60.0;

// height of middle intersection of slope down and up
base_height_midpoint = 25.0;

// height of the far end of the base away from the wall or stand
base_height_distal   = 70.0;

// length until the midpoint of slope down then up
base_length_midpoint = 40.0;

// length from proximal to distal.  Does not include tabs for inserts
base_length_distal   = 200.0;


/* [Wall Settings] */

/* [Stand Settings] */

/* [Inserts Settings] */

// hex insert outer radius
insert_r = 5.0;
insert_clearance = 0.13;
insert_tab_clearance = 0.13;


insert_tab_length = insert_r * 4;

use <./helpers.scad>

if (part == "all" || part == "handle")   { color("#0099dd") handle();  }
if (part == "all" || part == "base")     { color("#33ffee") base();    }
if (part == "all" || part == "wall")     { color("#ff9933") wall();    }
if (part == "all" || part == "stand")    { color("#666666") stand();   }
if (part == "all" || part == "inserts")  { color("#ff3366") inserts(); }

//#mov_x(-base_width/2)
//  cube([base_width, base_length_distal, base_height_proximal]);


/** Knife sharpener handle
 *
 * Includes the rod, sharpener stone holder, and handle.
 */
module handle() {
  if (show_key) { mov_y(-15) text("handle", halign="center", valign="center"); }
} // end of module handle()

/** Knife sharpener base
 *
 * The bottom of the knife sharpener and main stabilizing agent.  Holds the
 * knife, has holes for attaching the wall or stand, and has slots for magnets
 * for more easily holding the knife.
 */
module base() {
  base_tabs();
  if (show_key) { mov_y(-30) text("base", halign="center", valign="center"); }
} // end of module base()

/** A wall that attaches to the base
 *
 * Has many built-in holes with spherical joints for the knife sharpener
 * handle.  Each hole is intended to be a specific angle for sharpening.
 *
 * Use either the wall or the stand.
 */
module wall() {
  if (show_key) { mov_y(-45) text("wall", halign="center", valign="center"); }
} // end of module wall()

/** A stand that attaches to the base
 *
 * The stand has a single hole with a spherical joint.  It will have a way to
 * move or slide it up and down to the desired height.
 *
 * Use either the wall or the stand.
 */
module stand() {
  if (show_key) { mov_y(-60) text("stand", halign="center", valign="center"); }
} // end of module stand()

/** Hex-shaped insert rods to hold the base to either the stand or the wall
 *
 * Intended to be a tight fit in the holes in the base, stand, and wall.  At
 * the base of the insert is a foot piece so that it can be grasped and removed
 * when wanted.
 */
module inserts(clearance = 0.0) {
  r = insert_r;
  w = 4*r;
  dupe_x((base_width - w) / 2) mov_y(-2*clearance) insert(clearance);
  if (show_key) { mov_y(-75) text("inserts", halign="center", valign="center"); }
} // end of module inserts()


//
// Helper Functions
//

// one insert for the inserts() module
module insert(clearance = 0.0) {
  r      = insert_r + clearance;
  foot_h = (base_height_proximal / 5) - 2 * insert_tab_clearance + clearance;
  // if clearance is zero, let the length be a little short.
  L      = (clearance == 0.0) ? base_height_proximal - 1 : base_height_proximal;

  union() {
    mov_y( 2*r) cylinder(r=r, h=L, $fn=6);
    mov_x(-2*r) cube([4*r, r * (2 + sqrt(3)/2), foot_h]);
  }
} // end of module insert()

// insert tab without the hole, do holes afterwards
module insert_tab() {
  r       = insert_r;
  tab_clr = insert_tab_clearance;
  h       = (base_height_proximal / 5) - 2 * tab_clr;
  w       = 4*r;

  mov_x(-w/2) cube([w, w, h]);
} // end of module insert_tab()

// all tabs for the base object
module base_tabs() {
  tab_clr = insert_tab_clearance;
  dh      = (base_height_proximal / 5);
  h       = dh - 2 * tab_clr;
  r = insert_r;
  union() {
    dupe_x(base_width/2 - 2*r)
      mov_y(r * (2 + sqrt(3)/2) + insert_tab_clearance)
      insert_tab();
    dupe_x(base_width/2 - 2*r) mov_z(2*dh) insert_tab();
    dupe_x(base_width/2 - 2*r) mov_z(4*dh) insert_tab();
  }
} // end of module base_tabs()
