/* [Settings] */

prismatic_shaft_diameter = 7;
prismatic_shaft_length   = 25;
clearance_start          = 0.1;
clearance_step           = 0.05;
clearance_count          = 5;

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
