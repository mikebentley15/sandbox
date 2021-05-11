module M3(length, pitch=0.5) {
  head_height = 2.5;
  eps = 0.01;
  union() {
    translate([0, 0, head_height - eps])
      screw_shaft(3, length + eps, pitch);
    screw_hex_roundhead(6, head_height, 1);
  }
}

module M4(length, pitch=0.7) {
  head_height = 2.5;
  eps = 0.01;
  union() {
    translate([0, 0, head_height - eps])
      screw_shaft(4, length + eps, pitch);
    screw_hex_roundhead(7, head_height, 1.5);
  }
}

module M5(length, pitch=0.8) {
  head_height = 2.5;
  eps = 0.01;
  union() {
    translate([0, 0, head_height - eps])
      screw_shaft(5, length+eps, pitch);
    screw_hex_roundhead(9, head_height, 2.00);
  }
}

module M6(length, pitch=1.0) {
  head_height = 2.5;
  eps = 0.01;
  union() {
    translate([0, 0, head_height - eps])
      screw_shaft(6, length+eps, pitch);
    screw_hex_roundhead(10, head_height, 2.25);
  }
}

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
