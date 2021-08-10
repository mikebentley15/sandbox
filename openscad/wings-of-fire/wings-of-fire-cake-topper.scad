svg_scale         =  0.6;
svg_diameter      = 100.0;
height            = 50;
thickness         =  3;
svgfile           = "wings-of-fire.svg";
eps               = 0.02;
$fn               = 200;

use <./helpers.scad>

d = svg_diameter * svg_scale;
mid = d / 2;

linear_extrude(thickness) {
  svg(svgfile);
  translate([        6,     mid]) square([10,  5], center=true);
  translate([    d - 5,     mid]) square([10,  5], center=true);
  translate([mid - 2.5, -height]) square([ 5,  height + 10]);
}

module svg(filename) {
  scale([svg_scale, svg_scale, 1.0]) import(filename);
}
