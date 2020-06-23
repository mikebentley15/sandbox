/* [Mount Plate] */

// Width of the mount plate
mount_width = 50;

// Length of the mount plate
mount_length = 50;

// Height of the mount plate
mount_height = 5;

// Width distance to four mounting holes from center
mount_hole_x = 15;

// Length distance to four mounting holes from center
mount_hole_y = 15;

// Mount hole radius
mount_hole_radius = 3;



/* [Backbone] */

// Radius of the backbone
backbone_radius = 2.5;

// Backbone length
backbone_length = 200;



/* [Discs] */

// Number of discs
n_discs = 10;

disc_radius = 10;

disc_thickness = 3;



/* [Tendons] */

tendon_hole_radius = 2;

// Distance from backbone center
tendon_hole_distance = 7;

// Number of tendon holes
n_tendons = 3;



/* [Model Settings] */

// Extra length when making holes
eps = 0.04;

// How many edges for cylinder curves
cylinder_faces = 32;



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

difference() {
  tendon_robot();

  // base plate holes
  mov_xyz(mount_hole_x, mount_hole_y, mount_height / 2)
    cylinder(h=mount_height + eps,
             r=mount_hole_radius,
             center=true,
             $fn=cylinder_faces);
  mov_xyz(-mount_hole_x, mount_hole_y, mount_height / 2)
    cylinder(h=mount_height + eps,
             r=mount_hole_radius,
             center=true,
             $fn=cylinder_faces);
  mov_xyz(mount_hole_x, -mount_hole_y, mount_height / 2)
    cylinder(h=mount_height + eps,
             r=mount_hole_radius,
             center=true,
             $fn=cylinder_faces);
  mov_xyz(-mount_hole_x, -mount_hole_y, mount_height / 2)
    cylinder(h=mount_height + eps,
             r=mount_hole_radius,
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
               r=tendon_hole_radius,
               center=true,
               $fn=cylinder_faces);
  }
}
