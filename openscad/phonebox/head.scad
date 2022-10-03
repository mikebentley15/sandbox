sphere_radius = 875;
width         = 900;
height        = 360;

div           = 12;
eps           = 0.1;
$fn           = 200;

r = sphere_radius / div;
w = width / div;
h = height / div;

intersection() {
  translate([0,0, h/2 - r]) sphere(r=r);
  translate([0, 0, eps]) cube([w, w, h + eps], center=true);
}
