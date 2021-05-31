/* [Settings] */

// Results:
// - 0.30mm slides nice, but too much rotational play
// - 0.25mm is pretty good at sliding, but could use sanding.  It has less
//   rotational play.
// - 0.20mm does not fit at all
// - 0.15mm and 0.10mm: did not test because 0.20mm would not fit.
//
// Conclusion:
// - about 0.25mm clearance is good.  We could try 0.24, 0.23, and 0.22 before and
//   after sanding

prismatic_shaft_diameter = 7;
prismatic_shaft_length   = 25;
clearance_start          = 0.22;
clearance_step           = 0.01;
clearance_count          = 3;

use <./helpers.scad>

r = prismatic_shaft_diameter / 2;
$fn = 50;

for (i = [1:clearance_count]) {
  clearance = clearance_start + (i-1)*clearance_step;
  mov_y(1.5 * prismatic_shaft_diameter * (i - 1))
    mov_z((r - clearance) * sqrt(3)/2)
    difference() {
      rot_y(90)
      rot_z(30)
      cylinder(r=r - clearance,
               h=prismatic_shaft_length,
               $fn=6);
      mov_z(2.5)
        linear_extrude(5)
        mov_x(1.5)
        text(
          str(clearance),
          valign="center",
          size=2.5,
          font="Nimbus Mono PS:style=bold"
          );
  }
}
