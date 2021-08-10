svg_scale         =  0.6;
svg_diameter      = 100.0;
stamp_thickness   =  1.5;
height            =  3.5;
svgfile           = "wings-of-fire.svg";
eps               = 0.02;
$fn               = 200;

use <./helpers.scad>

d = svg_diameter * svg_scale;
mid = d / 2;

linear_extrude(stamp_thickness) translate([-mid, -mid]) svg(svgfile);
translate([0, 0, stamp_thickness - eps])
  cylinder(d=d, h=height + eps - stamp_thickness);

module svg(filename) {
  scale([svg_scale, svg_scale, 1.0]) import(filename);
}
