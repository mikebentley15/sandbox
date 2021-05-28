module M_screw(size, length, pitch=-1, profile=4) {
  assert(size == 3 || size == 4 || size == 5 || size == 6, "Unsupported size given");
  if (pitch == -1) { pitch = M_default_pitch(size); }
  head_height = M_screw_head_height(3);
  eps = 0.01;
  union() {
    translate([0, 0, head_height - eps])
      screw_shaft(size, length + eps, pitch, profile=profile);
    screw_hex_roundhead(
        M_screw_head_diameter(size),
        head_height,
        M_screw_hex_radius(size));
  }
}

module M_nut(size, pitch=-1, profile=4) {
  assert(size == 3 || size == 4 || size == 5 || size == 6, "Unsupported size given");
  if (pitch == -1) { pitch = M_default_pitch(size); }
  nut(size,
      M_nut_inner_diameter(size),
      pitch,
      M_nut_height(size),
      profile=profile);
}

module M_washer(size) {
  assert(size == 3 || size == 4 || size == 5 || size == 6, "Unsupported size given");
  washer(
    M_washer_outer_diameter(size),
    M_washer_inner_diameter(size),
    M_washer_thickness(size));
}

function M_screw_head_height(size) = 2.5;
function M_screw_head_diameter(size) =
  size == 3 ?  6 :
  size == 4 ?  7 :
  size == 5 ?  9 :
  size == 6 ? 10 : 0;
function M_screw_hex_radius(size) =
  size == 3 ? 1.00 :
  size == 4 ? 1.50 :
  size == 5 ? 2.00 :
  size == 6 ? 2.25 : 0;
function M_default_pitch(size) =
  size == 3 ? 0.5 :
  size == 4 ? 0.7 :
  size == 5 ? 0.8 :
  size == 6 ? 1.0 : 1.0;
function M_nut_height(size) =
  size == 3 ? 2 :
  size == 4 ? 3.25 :
  size == 5 ? 3.75 :
  size == 6 ? 5 : 5;
function hex_inner_diameter_to_outer_radius(d) = d / (2 * cos(30));
function M_nut_inner_diameter(size) =
  size == 3 ?  6 :
  size == 4 ?  7 :
  size == 5 ?  8 :
  size == 6 ? 10 : 0;
function M_nut_outer_radius(size) =
  hex_inner_diameter_to_outer_radius(M_nut_inner_diameter(size));
function M_washer_thickness(size) =
  size == 3 ? 0.65 :
  size == 4 ? 0.75 :
  size == 5 ? 0.75 :
  size == 6 ? 1.15 : 0;
function M_washer_outer_diameter(size) =
  size == 3 ?  7 :
  size == 4 ?  9 :
  size == 5 ? 10 :
  size == 6 ? 12 : 0;
function M_washer_inner_diameter(size) =
  size == 3 ? 3 :
  size == 4 ? 4 :
  size == 5 ? 5 :
  size == 6 ? 7 : 0;

module M3(length, pitch=0.5, profile=4) {
  M_screw(3, length, pitch, profile);
}

module M4(length, pitch=0.7, profile=4) {
  M_screw(4, length, pitch, profile);
}

module M5(length, pitch=0.8, profile=4) {
  M_screw(5, length, pitch, profile);
}

module M6(length, pitch=1.0, profile=4) {
  M_screw(6, length, pitch, profile);
}

module M3_nut(pitch=0.5, profile=4) {
  M_nut(3);
}

module M4_nut(pitch=0.7, profile=4) {
  M_nut(4);
}

module M5_nut(pitch=0.8, profile=4) {
  M_nut(5);
}

module M6_nut(pitch=1.0, profile=4) {
  M_nut(6);
}

module M3_washer() {
  M_washer(3);
}

module M4_washer() {
  M_washer(4);
}

module M5_washer() {
  M_washer(5);
}

module M6_washer() {
  M_washer(6);
}

/// Internal functions

module screw_hex_roundhead(width, height, hex_radius) {
  eps = 0.01;
  r = 0.8;
  difference() {
    rotate_extrude() {
      hull() {
          square([(width - 3) / 2, height]);
        translate([width / 2 - r, height - r, 0])
          circle(r);
      }
    }
    translate([0, 0, -eps])
      cylinder(r=hex_radius, h=height - 0.1 + eps, $fn=6);
  }
}

module screw_shaft(size, height, pitch, profile=4) {
  rotations = height / pitch;
  linear_extrude(height=height, twist=360*rotations)
    screwslice(size, pitch, profile=profile);
}

module nut(size, short_diameter, pitch, height, profile=4) {
  r = short_diameter / (2 * cos(30));
  eps = 0.01;
  b = 0.75 * sqrt(4 * height * height / (16 - pow(2 + sqrt(3), 2)));
  intersect_zscale = b / r;
  intersection() {
    difference() {
      cylinder(r=r, h=height, $fn=6);
      translate([0, 0, -eps])
        screw_shaft(size, height + 2*eps, pitch, profile=profile);
    }
    translate([0, 0, height/2])
      scale([1, 1, intersect_zscale])
      sphere(r=r);
  }
}

module washer(outer_diameter, inner_diameter, thickness) {
  ro = outer_diameter / 2;
  ri = inner_diameter / 2;
  eps = 0.01;
  difference() {
    cylinder(r=ro, h=thickness);
    translate([0, 0, -eps])
      cylinder(r=ri, h=thickness+2*eps);
  }
}

module screwslice(size, pitch, profile=4) {
  $fn = $fn < 3 ? 24 : $fn;
  H = .5 * pitch * pow(3, .5);
  r1 = size / 2 - H;
  d = 360 / $fn;
  p =
    [ for (i = [0:$fn]) [
      sin(d * i) * (r1 + (threadprofile(profile, d * i) * H)),
      cos(d * i) * (r1 + (threadprofile(profile, d * i) * H))]
    ];
  polygon(p);
}

function threadprofile(n, d) =
  n == 0 ? threadprofile_0(d) :
  n == 1 ? threadprofile_1(d) :
  n == 2 ? threadprofile_2(d) :
  n == 3 ? threadprofile_3(d) :
  n == 4 ? threadprofile_4(d) : 0;

function threadprofile_1(d) =
    (d180(d) > 157.5 ? 157.5 : d180(d)) / 180;

// Same as above, but flattens between 135 and 225 and flipped.
function threadprofile_0(d) =
    (d180(d) < 45 ? 45 : d180(d)) / 180;

function threadprofile_3(d) =
    d180(d) < 157.5 ? pow((d180(d) / 157.5), 2.718) * (157.5/180) :
    (d180(d) > 157.5 ? 157.5 : d180(d)) / 180;

function threadprofile_2(d) =
    d180(d) > 45 ? pow(((d180(d) - 45) / 135), 1/2.718) :
    (d180(d) < 45 ? 45 : d180(d)) / 180;

// idealized version
function threadprofile_4(d) = d180(d) / 180;

// Distance from 180 degrees
function d180(d) = abs(180 - (abs(d) % 360));
