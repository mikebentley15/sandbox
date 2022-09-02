// [Input File]

svg         = "headphone-cross-section.svg";
svg_height  = 10.8;
svg_width   = 29.6;

// [General Parameters]

thickness   =  3.5;
clearance   =  0.3;

height      =  8.0;
vertical_curve_radius = 88.5;

$fn         = 100;

use <helpers.scad>

theta = rad2deg(height / vertical_curve_radius);
rotate_extrude(angle = theta, convexity = 10) {
  translate([svg_height / 2 + vertical_curve_radius, 0])
  rotate([0, 0, -90])
  make_clip();
}

function rad2deg(x) = x * 180 / PI;

module make_clip() {
  difference() {
    shell2d(svg_width, svg_height, thickness, clearance)
      import(svg, center = true);
    difference() {
      polygon(points = [
        [0, 50.9],
        [-16, -20],
        [16, -20]
      ]);
      translate([-50, 0])
        square(100);
    }
  }
}

module shell2d(w, h, thickness, clearance) {
  outer_scaling = [
    (w + thickness + clearance) / w,
    (h + thickness + clearance) / h
  ];
  inner_scaling = [
    (w + clearance) / w,
    (h + clearance) / h
  ];
  difference() {
    scale(outer_scaling) children();
    scale(inner_scaling) children();
  }
}
