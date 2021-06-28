svg_scale         =  1.0;
svg_diameter      = 51.0;
stamp_height      =  1;
handle_height     = 30;
handle_diameter   = 60;
svgfile           = "logo-small.svg";
eps               = 0.02;
$fn               = 200;

translate(- svg_diameter / 2 * [1, 1, 0])
  translate(- stamp_height * [0, 0, 1])
  linear_extrude(stamp_height + eps)
  svg(svgfile);

difference() {
  rotate([0, 0, 22.5])
    cylinder(d = handle_diameter, h = handle_height, $fn = 8);

  //dupe(dx = 2 * handle_diameter * cos(22.5))
  //  move(dz = handle_height / 2)
  //  scale([1, 1, handle_height / (handle_diameter * sin(22.5))])
  //  sphere(30);

  dupe(dx = 2 * handle_diameter * cos(22.5))
    move(dz = handle_height / 2)
    scale([1, 1, handle_height / (handle_diameter * sin(22.5))])
    rotate([90, 0, 0])
    cylinder(r = 30, h = handle_diameter, center = true);
}


module move(dx=0, dy=0, dz=0) { translate([dx,dy,dz]) children(); }

module dupe(dx=0, dy=0, dz=0) {
  translate(-[dx,dy,dz] / 2) children();
  translate([dx,dy,dz] / 2) children();
}

module svg(filename) {
  scale([svg_scale, svg_scale, 1.0]) import(filename);
}
