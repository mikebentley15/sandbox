module M_screw(size, length, pitch=-1, profile=4, simplify=false) {
  //assert(size == 3 || size == 4 || size == 5 || size == 6, "Unsupported size given");
  used_pitch = pitch == -1 ? M_default_pitch(size) : pitch;
  head_height = M_screw_head_height(size);
  eps = 0.01;
  union() {
    translate([0, 0, head_height - eps])
      screw_shaft(size, length + eps, used_pitch, profile=profile,
                  simplify=simplify);
    screw_hex_roundhead(
        M_screw_head_diameter(size),
        head_height,
        M_screw_hex_radius(size),
        simplify=simplify);
  }
}

module M_nut(size, pitch=-1, profile=4, simplify=false) {
  //assert(size == 3 || size == 4 || size == 5 || size == 6, "Unsupported size given");
  used_pitch = pitch == -1 ? M_default_pitch(size) : pitch;
  nut(size,
      M_nut_inner_diameter(size),
      used_pitch,
      M_nut_height(size),
      profile=profile,
      simplify=simplify);
}

module M_washer(size) {
  //assert(size == 3 || size == 4 || size == 5 || size == 6, "Unsupported size given");
  washer(
    M_washer_outer_diameter(size),
    M_washer_inner_diameter(size),
    M_washer_thickness(size));
}

// uses lookup() function to linearly interpolate between sizes
function M_screw_head_height(size) = lookup(size, [
    [3, 1.6],
    [4, 2.4],
    [5, 2.8],
    [6, 3.3]
  ]);
function M_screw_head_diameter(size) = lookup(size, [
    [3,  5.56],
    [4,  7.45],
    [5,  9.4 ],
    [6, 10.3 ]
  ]);
function M_screw_hex_radius(size) = lookup(size, [
    [3, 1.00],
    [4, 1.25],
    [5, 1.75],
    [6, 2.25]
  ]);
function M_default_pitch(size) = lookup(size, [
    [3, 0.5],
    [4, 0.7],
    [5, 0.8],
    [6, 1.0]
  ]);
function M_nut_height(size) = lookup(size, [
    [3, 2.1 ],
    [4, 2.9 ],
    [5, 3.8 ],
    [6, 4.92]
  ]);
function hex_inner_diameter_to_outer_radius(d) = d / (2 * cos(30));
function M_nut_inner_diameter(size) = lookup(size, [
    [3, 5.2 ],
    [4, 6.7 ],
    [5, 7.9 ],
    [6, 9.83]
  ]);
function M_nut_outer_radius(size) =
  hex_inner_diameter_to_outer_radius(M_nut_inner_diameter(size));
function M_washer_thickness(size) = lookup(size, [
    [3, 0.62],
    [4, 0.95],
    [5, 1.00],
    [6, 1.05]
  ]);
function M_washer_outer_diameter(size) = lookup(size, [
    [3,  6.83],
    [4,  8.8 ],
    [5,  9.9 ],
    [6, 11.7 ]
  ]);
function M_washer_inner_diameter(size) = lookup(size, [
    [3, 3.2],
    [4, 4.4],
    [5, 5.3],
    [6, 6.5]
  ]);

module M3(length, pitch=0.5, profile=4, simplify=false) {
  M_screw(3, length, pitch, profile, simplify);
}

module M4(length, pitch=0.7, profile=4, simplify=false) {
  M_screw(4, length, pitch, profile, simplify);
}

module M5(length, pitch=0.8, profile=4, simplify=false) {
  M_screw(5, length, pitch, profile, simplify);
}

module M6(length, pitch=1.0, profile=4, simplify=false) {
  M_screw(6, length, pitch, profile, simplify);
}

module M3_nut(pitch=0.5, profile=4, simplify=false) {
  M_nut(3, pitch, profile, simplify);
}

module M4_nut(pitch=0.7, profile=4, simplify=false) {
  M_nut(4, pitch, profile, simplify);
}

module M5_nut(pitch=0.8, profile=4, simplify=false) {
  M_nut(5, pitch, profile, simplify);
}

module M6_nut(pitch=1.0, profile=4, simplify=false) {
  M_nut(6, pitch, profile, simplify);
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

module screw_hex_roundhead(width, height, hex_radius, simplify=false) {
  eps = 0.01;
  r = 0.8;
  difference() {
    if (simplify) {
      union() {
        translate([0, 0, height - r]) cylinder(d=width, h=r);
        cylinder(d1=(width - 3), d2=width, h=height - r);
      }
    } else {
      rotate_extrude() {
        hull() {
            square([(width - 3) / 2, height]);
          translate([width / 2 - r, height - r, 0])
            circle(r);
        }
      }
    }
    translate([0, 0, -eps])
      cylinder(r=hex_radius, h=height - 0.1 + eps, $fn=6);
  }
}

module screw_shaft(size, height, pitch, profile=4, simplify=false) {
  if (simplify) {
    cylinder(d=size, h=height);
  } else {
    rotations = height / pitch;
    linear_extrude(height=height, twist=-360*rotations)
      screwslice(size, pitch, profile=profile);
  }
}

module nut(size, short_diameter, pitch, height, profile=4, simplify=false) {
  r = short_diameter / (2 * cos(30));
  eps = 0.01;
  b = 0.75 * sqrt(4 * height * height / (16 - pow(2 + sqrt(3), 2)));
  intersect_zscale = b / r;
  intersection() {
    difference() {
      cylinder(r=r, h=height, $fn=6);
      translate([0, 0, -eps])
        screw_shaft(size, height + 2*eps, pitch, profile=profile,
                    simplify=simplify);
    }
    if (!simplify) {
      translate([0, 0, height/2])
        scale([1, 1, intersect_zscale])
        sphere(r=r);
    }
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
  if (profile == 5) {
    union() {
      circle(d = 0.85 * size);
      difference() {
        circle(d = size);
        translate([- 0.75 * size, 0, 0]) square(1.5 * size);
      }
    }
  } else {
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
