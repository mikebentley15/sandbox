svg_scale         =  0.3;
svg_diameter      = 100.0;
height            = 50;
thickness         =  3;
coin_border       =  2;
svgfile           = "wings-of-fire.svg";
eps               = 0.02;
$fn               = 200;

use <./helpers.scad>

d = svg_diameter * svg_scale;
mid = d / 2;

difference() {
  cylinder(d=d + 2 * coin_border, h=thickness);
  mov_z(thickness / 2 + eps)
    linear_extrude(thickness / 2) translate(-mid * [1,1]) svg(svgfile);
}

module svg(filename) {
  scale([svg_scale, svg_scale, 1.0]) import(filename);
}
