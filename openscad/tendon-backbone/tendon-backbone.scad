/* [Mount Plate] */

// Width of the mount plate (mm)
mount_width = 50.0;

// Length of the mount plate (mm)
mount_length = 50.0;

// Height of the mount plate (mm)
mount_height = 5.0;

// Width distance to four mounting holes from center (mm)
mount_hole_x = 15.0;

// Length distance to four mounting holes from center (mm)
mount_hole_y = 15.0;

// Mount hole radius (mm)
mount_hole_radius = 2.0;



/* [Backbone] */

// Radius of the backbone (mm)
backbone_radius = 2.0;

// Backbone length (mm)
backbone_length = 200.0;



/* [Discs] */

// Number of discs
n_discs = 9;

disc_radius = 10.0;

disc_thickness = 2.0;



/* [Tendons] */

tendon_hole_radius = 0.6;

// Distance from backbone center (mm)
tendon_hole_distance = 8.0;

// Number of tendon holes
n_tendons = 12;



/* [Model Settings] */

// Standing or laying down
standing = true;

// Extra length when making holes (mm)
eps = 0.01;

// How many edges for cylinder curves
cylinder_faces = 32;



/* [Built-in Printing Supports] */

// Enables sacraficial bridging for holes to print above
sacraficial_bridging = true;

// Used for sacraficial bridging (mm)
sacraficial_bridging_height = 0.2;

// Enables thin overhang knockout printing tabs
overhang_support = true;

// Width of thin overhang (mm)
overhang_width = 0.5;


/*** End of settings ***/



use <./helpers.scad>

module tendon_robot() {
  union() {
    // mount plate
    mov_xyz(-mount_width / 2, -mount_length / 2, 0)
      cube(size=[mount_width, mount_length, mount_height]);

    // backbone
    mov_xyz(0, 0, backbone_length / 2 + mount_height)
      cylinder(h=backbone_length + eps,
               r=backbone_radius,
               center=true,
               $fn=cylinder_faces);

    // discs
    disc_separation = backbone_length / n_discs;
    for (i = [1:n_discs]) {
      mov_xyz(0, 0, mount_height + (i * disc_separation))
        cylinder(h=disc_thickness,
                 r=disc_radius,
                 center=true,
                 $fn=cylinder_faces);
    }
  }
}

module tendon_robot_holes() {
  union() {
    // base plate holes
    mov_xyz(mount_hole_x, mount_hole_y, mount_height / 2)
      cylinder(h=mount_height + eps,
               r=outer_radius(mount_hole_radius, cylinder_faces),
               center=true,
               $fn=cylinder_faces);
    mov_xyz(-mount_hole_x, mount_hole_y, mount_height / 2)
      cylinder(h=mount_height + eps,
               r=outer_radius(mount_hole_radius, cylinder_faces),
               center=true,
               $fn=cylinder_faces);
    mov_xyz(mount_hole_x, -mount_hole_y, mount_height / 2)
      cylinder(h=mount_height + eps,
               r=outer_radius(mount_hole_radius, cylinder_faces),
               center=true,
               $fn=cylinder_faces);
    mov_xyz(-mount_hole_x, -mount_hole_y, mount_height / 2)
      cylinder(h=mount_height + eps,
               r=outer_radius(mount_hole_radius, cylinder_faces),
               center=true,
               $fn=cylinder_faces);

    // tendon holes
    d_rot = 360 / n_tendons;
    for (i = [1:n_tendons]) {
      hole_height = mount_height + backbone_length + disc_thickness + eps;
      rot = i * d_rot;
      mov_xyz(tendon_hole_distance * cos(rot),
              tendon_hole_distance * sin(rot),
              hole_height / 2 - eps)
        cylinder(h=hole_height,
                 r=center_radius(tendon_hole_radius, cylinder_faces),
                 center=true,
                 $fn=cylinder_faces);
    }
  }
}

module tendon_robot_with_holes() {
  difference() {
    tendon_robot();
    tendon_robot_holes();
  }
}

module standing_sacraficial_bridging() {
  disc_separation = backbone_length / n_discs;
  for (i = [1:n_discs]) {
    mov_z(mount_height + (i * disc_separation) - disc_thickness / 2
          - sacraficial_bridging_height / 2)
      cylinder(h=sacraficial_bridging_height,
               r=disc_radius,
               center=true,
               $fn=cylinder_faces);
  }
}

module laying_sacraficial_bridging() {
  box_x = backbone_radius / 2;
  box_y = backbone_length + 2 * eps;
  box_z = sacraficial_bridging_height;
  mov_y(mount_height - eps)
    mov_x(-overhang_width / 2)
    mov_z(mount_length / 2 - backbone_radius - box_z)
    cube([box_x, box_y, box_z + eps]);
}

module standing_overhang_support() {
  mov_z(mount_height + backbone_length / 2)
    difference() {
      cylinder(h=backbone_length,
               r=disc_radius,
               center=true,
               $fn=cylinder_faces);
      cylinder(h=backbone_length + eps,
               r=disc_radius - overhang_width,
               center=true,
               $fn=cylinder_faces);
      cube([disc_radius * 2 + eps,
            overhang_width * 2,
            backbone_length + eps],
           center=true);
      cube([overhang_width * 2,
            disc_radius * 2 + eps,
            backbone_length + eps],
           center=true);
    }
}

module laying_overhang_support() {
  box_width = disc_radius / 4;
  box_length = disc_thickness;
  box_height = mount_length / 2 - disc_radius;
  disc_separation = backbone_length / n_discs;
  for (i = [1:n_discs]) {
    mov_y(mount_height + (i * disc_separation))
      mov_z(box_height / 2)
      difference() {
        cube([box_width,
              disc_thickness,
              box_height],
             center=true);
        cube([box_width - 2 * overhang_width,
              disc_thickness - 2*overhang_width,
              box_height + eps],
             center=true);
      }
    mov_y(mount_height + (i * disc_separation))
      mov_z(overhang_width)
      cube([box_width * 3, box_length * 3, overhang_width * 2], center=true);
  }
}

if (standing) {
  union() {
    tendon_robot_with_holes();
    if (sacraficial_bridging) {
      standing_sacraficial_bridging();
    }
    if (overhang_support) {
      standing_overhang_support();
    }
  }
} else {
  union() {
    mov_z(mount_length / 2)
      rot_x(-90)
      tendon_robot_with_holes();
    if (sacraficial_bridging) {
      laying_sacraficial_bridging();
    }
    if (overhang_support) {
      laying_overhang_support();
    }
  }
}
