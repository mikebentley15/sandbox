use <./helpers.scad>

$fn = 15;

for (i = [0:5]) {
  mov_x(10*i + i*i) minkowski() { shape_1(); sphere(i); }
}

for (i = [0:5]) {
  mov_y(25) mov_x(10*i + i*i) minkowski() { shape_2(); sphere(i); }
}

for (i = [0:5]) {
  mov_y(50) mov_x(10*i + i*i) minkowski() { shape_3(); sphere(i); }
}

module shape_1() {
  cube(5);
}

module shape_2() {
  difference() {
    shape_1();
    translate([2.5, 2.5, -0.5]) cylinder(r = 2, h = 6, $fn = 30);
  }
}

module shape_3() {
  shape_1();
  translate([2.5, 2.5, 4.5]) cylinder(r = 1, h = 3.5, $fn = 30);
}
