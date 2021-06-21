/* [General Settings] */

// Which model to render
part = "all"; // ["all", "motor-mount", "sensor-mount", "L-bind", "bearing-mount", "motor-prismatic-coupler", "needle-prismatic-coupler"]

/* [Bounding Boxes] */

// bottom platform
show_platform_bounding_box = false;

// all L brackets together
show_L_brackets_bounding_box = false;

// the vertical slit for the L brackets
show_L_brackets_vsplit_bounding_box = false;

// motor with shaft
show_motor_bounding_box = false;

// the box part of the motor
show_motor_box_bounding_box = false;

// printed mount for the motor to the L brackets
show_motor_mount_bounding_box = false;

// full force sensor with protuberances on the sides
show_sensor_bounding_box = false;

// inner aluminum body of the force sensor
show_sensor_inner_bounding_box = false;

// printed mounting for the force sensor
show_sensor_mount_bounding_box = false;

// printed mounting for the force sensor (part around L brackets)
show_sensor_mount_bracket_part_bounding_box = false;

// printed mounting for the force sensor (part around force sensor)
show_sensor_mount_sensor_part_bounding_box = false;


/* [Printed Motor Mount] */

motor_mount_front_buffer       =  4;
motor_mount_depth_buffer       =  5;
motor_mount_height             = 50;
motor_mount_shaft_clearance    =  0.7;
motor_mount_cylinder_clearance =  0.2;
motor_mount_bracket_clearance  =  0.2;
motor_mount_screw_clearance    =  0.2;


/* [Printed Sensor Mount] */

sensor_mount_height                 = 28;
sensor_mount_depth_buffer           =  5;
sensor_mount_width_buffer           =  4;
sensor_mount_bracket_clearance      =  0.2;
sensor_mount_screw_clearance        =  0.2;
sensor_mount_screw_size             =  5;
sensor_mount_nut_depth              =  2;
sensor_mount_sensor_clearance       =  0.4;
sensor_mount_sensor_wall            =  3;
sensor_mount_sensor_wall_slit_width =  8;
// vertical position (0 = bottom)
sensor_mount_z_offset               =  0;


/* [Printed L-Binder] */

L_binder_width             = 10;
L_binder_bracket_clearance = 0.2;
L_binder_wall_thickness    = 2;


/* [Printed Bearing Mount] */

bearing_mount_sensor_wall            =  4;
bearing_mount_sensor_clearance       =  0.2;
bearing_mount_sensor_overhang        =  2;
bearing_mount_sensor_overhang_height =  2;
bearing_mount_sensor_wall_length     = 10;
bearing_mount_screw_size             = 4;
bearing_mount_screw_clearance        = 0.2;
bearing_mount_bearing_clearance      = 0.15;
// extra buffer away from the sensor mount
bearing_mount_bearing_sensor_x_buffer = 5;
bearing_mount_bearing_wall_thickness = 3;
bearing_mount_bearing_wall_height    = 5;
bearing_mount_depth                  = 60;
bearing_mount_center_hole_clearance  = 2;
// TODO: others


/* [Printed Motor Prismatic Coupler] */

motor_coupler_length            = 15;
motor_coupler_diameter          = 20;
motor_coupler_screw_offset      =  6;
motor_coupler_screw_size        =  3; // [3, 4]
motor_coupler_screw_clearance   =  0.2;
motor_coupler_shaft_clearance   =  0.3;
motor_coupler_slit_size         =  1.3;
// thickness on both sides of the slit for the screw
motor_coupler_clamp_thickness   =  3;
motor_prismatic_length          = 20;
motor_prismatic_cavity_depth    = 17;
motor_prismatic_outer_diameter  = 10;
motor_prismatic_inner_diameter  =  7;
motor_coupler_mount_x_offset    =  5;


/* [Printed Needle Prismatic Coupler] */


/* [Platform] */

platform_width  = 60;
platform_height = 32;
platform_depth  = 42;

platform_side_screw_side_distance = 10;
platform_side_screw_top_distance  = 6;

platform_top_screw_width_side_distance = 15;
platform_top_screw_depth_side_distance = 6;
platform_top_screw_hole_height         = 16;

platform_screw_size = 4;


/* [L Bracket] */

L_bracket_height    = 120;
L_bracket_length    = 99.3;
L_bracket_width     = 24.1;
L_bracket_thickness = 2.7;

L_bracket_slit_width         = 5.2;
L_bracket_slit_end_distance  = 10.3;
L_bracket_bottom_slit_length = 69.8;
L_bracket_side_slit_length   = 89.8;


/* [Motor] */

motor_length = 47.3;
motor_depth  = 42;
motor_height = 42;

motor_cylinder_length     = 1.7;
motor_cylinder_diameter   = 21.9;
motor_shaft_length        = 24;
motor_shaft_diameter      = 4.9;
motor_shaft_cutout_length = 14.7;
motor_shaft_cutout_height = 4.5;

motor_screw_size          = 3;
motor_screw_depth         = 4.5;
motor_screw_side_distance = 5.5;
motor_screw_top_distance  = 5.5;

motor_mount_x_buffer      = 20;
motor_z_offset            = 20;


/* [Force Sensor] */

sensor_height    = 80.15;
sensor_width     = 12.6;
sensor_depth     = 12.6;

sensor_center_hole_diameter = 10.63;
sensor_center_hole_offset   = 3.56;

sensor_center_bulge_thickness = 1.18;
sensor_center_bulge_length    = 23.5;
sensor_center_bulge_depth     = 12.6;
sensor_right_bulge_thickness  = 1.6;

sensor_bottom_screw_size      = 5;
sensor_bottom_screw_offset_1  = 5;
sensor_bottom_screw_offset_2  = 20;
sensor_top_screw_size         = 4;
sensor_top_screw_offset_1     = 5;
sensor_top_screw_offset_2     = 20;


/* [Bearing] */

bearing_outer_diameter = 22;
bearing_inner_diameter =  8;
bearing_thickness      =  7;


/* [General Print Settings] */

sacrificial_bridging                = true;
first_layer_height                  = 0.2;
layer_height                        = 0.2;
extrusion_width                     = 0.45;

/* [Advanced Settings] */

eps = 0.02;
$fn = 20;


/* [Colors] */

platform_color     = "#ffe683aa";
bracket_color      = "#ddddddaa";
screw_color        = "#777777aa";
nut_color          = "#3333aaaa";
washer_color       = "#333333aa";
force_sensor_color = "#6cff6caa";
motor_color        = "#00ffffaa";
bearing_color      = "#ff00ffaa";
printed_color_1    = "#ff0000aa";




//
// helpful calculated distances
//

sensor_bottom_screw_distance =
    sensor_bottom_screw_offset_2 - sensor_bottom_screw_offset_1;
sensor_top_screw_distance =
    sensor_top_screw_offset_2 - sensor_top_screw_offset_1;



// implementation

use <./helpers.scad>
use <screwlib.scad>

//---------------------------//
// bounding boxes of objects //
//---------------------------//
//
// Idea is to have each individual object generated by a module such that its
// center (at least to its bounding box) is at the origin.  The main level will
// then translate each object to their location.
//
// Maybe this is a good idea.  But ultimately, it's more important to
// precompute the bounding boxes of each object (and locations) so that further
// object locations can be specified relative to other bounding boxes.
//
// For now, we are assuming that the rotation of each object can be determined
// independently of other objects.

platform_bb = bb(
    center=[platform_width/2, platform_depth/2, platform_height/2],
    //center = [0, 0, 0],
    dim = [platform_width, platform_depth, platform_height]
  );

L_brackets_bb = bb(
    center = [
      bb_xcenter(platform_bb),
      bb_ycenter(platform_bb),
      bb_zmax(platform_bb)
        + (L_bracket_height + L_bracket_thickness) / 2
    ],
    dim = [
      L_bracket_length + L_bracket_thickness,
      L_bracket_width
        + platform_depth
        - 2 * platform_top_screw_depth_side_distance,
      L_bracket_height + L_bracket_thickness
    ]
  );

// vertical slit in L_brackets() (going through both side brackets)
// Note: -x side has a lower slit than the +x side
// TODO: switch which L_bracket is on top; we want the force sensor to be able
//       to be mounted lower.
L_brackets_vsplit_bb = bb(
    center = bb_center(L_brackets_bb) + [
      0,
      0,
      bb_zdim(L_brackets_bb) / 2
        - L_bracket_side_slit_length / 2
        - L_bracket_thickness / 2
        - L_bracket_slit_end_distance
    ],
    dim = [
      bb_xdim(L_brackets_bb),
      bb_ydim(L_brackets_bb)
        - L_bracket_width
        + L_bracket_slit_width,
      L_bracket_side_slit_length
        + L_bracket_thickness
    ]
  );

L_bracket_inbetween_space = bb_ydim(L_brackets_bb) - 2 * L_bracket_width;

// specifically for the box part of the motor
motor_box_bb = bb(
    center = [
      bb_xmin(L_brackets_bb)
        - motor_length / 2
        - motor_cylinder_length,
      bb_ycenter(L_brackets_bb),
      bb_zmin(L_brackets_vsplit_bb)
        + motor_screw_size / 2
        + motor_height / 2
        - motor_screw_top_distance
        + motor_z_offset
    ],
    dim = [
      motor_length,
      motor_depth,
      motor_height
    ]
  );

// for the full motor, including box and shaft
motor_bb = bb(
    center = bb_center(motor_box_bb) + [motor_shaft_length / 2, 0, 0],
    dim = bb_dim(motor_box_bb) + [motor_shaft_length, 0, 0]
  );

motor_mount_bb = bb(
    center = [
      bb_xmax(motor_box_bb)
        + motor_cylinder_length / 2
        + L_bracket_thickness / 2
        + motor_mount_front_buffer / 2,
      bb_ycenter(motor_box_bb),
      bb_zcenter(motor_box_bb)
    ],
    dim = [
      motor_cylinder_length
        + L_bracket_thickness
        + motor_mount_front_buffer,
      bb_ydim(L_brackets_bb)
        + 2 * motor_mount_bracket_clearance
        + 2 * motor_mount_depth_buffer,
      motor_mount_height
    ]
  );

sensor_mount_bracket_part_bb = bb(
    center = [
      bb_xmax(L_brackets_bb)
        - L_bracket_thickness / 2,
      bb_ycenter(L_brackets_bb),
      bb_zmin(L_brackets_vsplit_bb)
        + L_bracket_thickness
        + sensor_mount_screw_size / 2
        + sensor_mount_screw_clearance
        + sensor_bottom_screw_distance / 2
        + sensor_mount_z_offset
    ],
    dim = [
      L_bracket_thickness
        + 2 * sensor_mount_width_buffer
        + 2 * sensor_mount_bracket_clearance,
      bb_ydim(L_brackets_bb)
        + 2 * sensor_mount_depth_buffer
        + 2 * sensor_mount_bracket_clearance,
      sensor_mount_height
    ]
  );

// force sensor without the bulges, just the main aluminum body
sensor_inner_bb = bb(
    center = [
      bb_xmin(sensor_mount_bracket_part_bb)
        - sensor_width / 2
        - sensor_mount_sensor_clearance,
      bb_ymin(L_brackets_vsplit_bb)
        + L_bracket_slit_width / 2,
      // align so the bottom screws align with the sensor mount bracket
      bb_zcenter(sensor_mount_bracket_part_bb)
        + sensor_height / 2
        - sensor_bottom_screw_offset_1
        - sensor_bottom_screw_distance / 2
    ],
    dim = [
      sensor_width,
      sensor_depth,
      sensor_height
    ]
  );

sensor_bb = bb(
    center = bb_center(sensor_inner_bb)
        + [0, -sensor_right_bulge_thickness / 2, 0],
    dim = bb_dim(sensor_inner_bb)
        + [2 * sensor_center_bulge_thickness, sensor_right_bulge_thickness, 0]
  );

sensor_mount_sensor_part_bb = bb(
    center = [
      bb_xcenter(sensor_bb)
        - sensor_mount_sensor_wall / 2,
      bb_ycenter(sensor_bb),
      bb_zcenter(sensor_mount_bracket_part_bb)
    ],
    dim = [
      bb_xdim(sensor_inner_bb)
        + 2 * sensor_mount_sensor_clearance
        + sensor_mount_sensor_wall,
      bb_ydim(sensor_bb)
        + 2 * sensor_mount_sensor_clearance
        + 2 * sensor_mount_sensor_wall,
      bb_zdim(sensor_mount_bracket_part_bb)
    ]
  );

sensor_mount_bb = bb_join(sensor_mount_bracket_part_bb,
                          sensor_mount_sensor_part_bb);

if (part == "all") {
  translate(bb_center(platform_bb)) platform();
  translate(bb_center(L_brackets_bb)) L_brackets();
  L_brackets_screws();
  translate(bb_center(motor_bb)) motor();
  translate(bb_center(motor_mount_bb)) motor_mount();
  motor_mount_screws();
  translate(bb_center(sensor_bb)) force_sensor();
  translate(bb_center(sensor_mount_bb)) sensor_mount();
  sensor_mount_screws();
  //mounted_L_binders();
  //mounted_motor_coupler();
  //bearing();
  //mounted_bearing_mount();
}

if (part == "motor-mount") {
  motor_mount(sacrificial_bridging=sacrificial_bridging);
}

if (part == "sensor-mount") {
  sensor_mount();
}

if (part == "L-bind") {
  L_binder();
}

if (part == "bearing-mount") {
  bearing_mount();
}

if (part == "motor-prismatic-coupler") {
  motor_coupler();
}

if (part == "needle-prismatic-coupler") {

}


check_show_bb(show_platform_bounding_box, platform_bb);
check_show_bb(show_L_brackets_bounding_box, L_brackets_bb);
check_show_bb(show_L_brackets_vsplit_bounding_box, L_brackets_vsplit_bb);
check_show_bb(show_motor_box_bounding_box, motor_box_bb);
check_show_bb(show_motor_bounding_box, motor_bb);
check_show_bb(show_motor_mount_bounding_box, motor_mount_bb);
check_show_bb(show_sensor_inner_bounding_box, sensor_inner_bb);
check_show_bb(show_sensor_bounding_box, sensor_bb);
check_show_bb(show_sensor_mount_bracket_part_bounding_box,
              sensor_mount_bracket_part_bb);
check_show_bb(show_sensor_mount_sensor_part_bounding_box,
              sensor_mount_sensor_part_bb);
check_show_bb(show_sensor_mount_bounding_box, sensor_mount_bb);

module check_show_bb(cond, bb) {
  if (cond) {
    show_bb(bb);
  }
}

module platform() {
  color(platform_color)
  difference() {
    cube(bb_dim(platform_bb), center = true);
    // side screw holes
    mov_z(platform_height/2 - platform_side_screw_top_distance)
      dupe_x(
          platform_width
            - 2 * platform_side_screw_side_distance
        )
      rot_x(-90)
      cylinder(r=platform_screw_size/2, h=platform_depth+2*eps, center=true);
    // top screw holes
    mov_z(platform_height / 2 - platform_top_screw_hole_height)
      dupe_x(
          platform_width
            - 2 * platform_top_screw_width_side_distance
        )
      dupe_y(
          platform_depth
            - 2 * platform_top_screw_depth_side_distance
        )
      cylinder(r=platform_screw_size/2, h=platform_top_screw_hole_height+eps);
  }
}

// screws for both pairs of L brackets to the platform
module L_brackets_screws() {
  translate([
      bb_xcenter(L_brackets_bb),
      bb_ycenter(L_brackets_bb),
      bb_zmin(L_brackets_bb)
        + 2 * L_bracket_thickness
    ])
    dupe_x(
      platform_width
        - 2 * platform_top_screw_width_side_distance
      )
    dupe_y(
      platform_depth
        - 2 * platform_top_screw_depth_side_distance
      )
    union() {
      color(washer_color)
        M4_washer();
      color(screw_color)
        mov_z(M_screw_head_height(4))
        rot_x(180)
        M4(12);
    }
}

// all four L brackets aligned
module L_brackets() {
  dupe_y(bb_ydim(L_brackets_bb) - L_bracket_width)
  mir_x(dx=L_bracket_thickness, dz=L_bracket_thickness)
    L_bracket();
}

// single L-bracket near the origin
module L_bracket() {
  big_r = L_bracket_width / 2;
  color(bracket_color)
  translate(-.5*[L_bracket_length, L_bracket_width, L_bracket_height])
  difference() {
    union() {
      cube([L_bracket_length-big_r, L_bracket_width, L_bracket_thickness]);
      cube([L_bracket_thickness, L_bracket_width, L_bracket_height-big_r]);
      translate([L_bracket_length-big_r, big_r, 0])
        cylinder(r=big_r, h=L_bracket_thickness);
      translate([0, big_r, L_bracket_height-big_r])
        rot_y(90)
        cylinder(r=big_r, h=L_bracket_thickness);
    }
    translate([
        L_bracket_length - L_bracket_bottom_slit_length - L_bracket_slit_end_distance,
        (L_bracket_width - L_bracket_slit_width) / 2,
        -eps
      ])
      L_bracket_slit(
          L_bracket_bottom_slit_length,
          L_bracket_slit_width,
          L_bracket_thickness + 2*eps
        );
    translate([
        -eps,
        (L_bracket_width - L_bracket_slit_width) / 2,
        L_bracket_height - L_bracket_slit_end_distance
      ])
      rot_y(90)
      L_bracket_slit(
          L_bracket_side_slit_length,
          L_bracket_slit_width,
          L_bracket_thickness + 2*eps
        );
  }
}

module L_bracket_slit(length, width, thickness) {
  r = width / 2;
  union() {
    mov_x(r) cube([length - width, width, thickness]);
    mov_xy(r, r) cylinder(r=r, h=thickness);
    mov_xy(length - r, r) cylinder(r=r, h=thickness);
  }
}

module motor() {
  color(motor_color)
  mov_x(- motor_shaft_length / 2) // center offset because of the shaft
  difference() {
    union() {
      // main body
      intersection() {
        cube(bb_dim(motor_box_bb), center=true);
        rot_x(45)
          cube(bb_dim(motor_box_bb) + [2*eps, 10, 10], center=true);
      }

      // cylinder
      mov_x(motor_length / 2 - eps)
        rot_y(90)
        cylinder(r=motor_cylinder_diameter/2, h=motor_cylinder_length+eps);

      // shaft
      mov_x(motor_length / 2)
        rot_y(90)
        difference() {
          cylinder(d=motor_shaft_diameter, h=motor_shaft_length);
          translate([
            -motor_shaft_diameter / 2 - eps,
            -motor_shaft_diameter / 2,
            motor_shaft_length - motor_shaft_cutout_length
            ])
            cube([
                motor_shaft_diameter - motor_shaft_cutout_height + eps,
                motor_shaft_diameter,
                motor_shaft_cutout_length + eps
              ]);
        }
    }

    // screw holes
    mov_x(motor_length / 2 - motor_screw_depth)
      dupe_y(motor_depth - 2 * motor_screw_side_distance)
      dupe_z(motor_height - 2 * motor_screw_top_distance)
      rot_y(90)
      cylinder(d=motor_screw_size, h=motor_screw_depth+eps);
  }
}

module motor_mount(sacrificial_bridging=false) {
  color(printed_color_1)
  union() {
    if (sacrificial_bridging) { motor_mount_sacrificial_bridging(); }
    difference() {
      // main body
      cube(bb_dim(motor_mount_bb), center=true);

      // shaft hole
      rot_y(90)
        cylinder(
            r = motor_shaft_diameter / 2 + motor_mount_shaft_clearance,
            h = bb_xdim(motor_mount_bb) + 2 * eps,
            center = true
          );

      // motor cylinder slide in
      mov_x(-bb_xdim(motor_mount_bb) / 2 - eps)
        rot_y(90)
        cylinder(
            r = motor_cylinder_diameter / 2 + motor_mount_shaft_clearance,
            h = motor_cylinder_length + motor_mount_cylinder_clearance + eps
          );

      // bracket slide-in holes
      mov_x(- bb_xdim(motor_mount_bb) / 2
            + motor_cylinder_length
            + L_bracket_thickness / 2
            + motor_mount_bracket_clearance)
        dupe_y(
            L_bracket_width
            + L_bracket_inbetween_space
          )
        cube([
            L_bracket_thickness + 2 * motor_mount_bracket_clearance,
            L_bracket_width + 2 * motor_mount_bracket_clearance,
            motor_mount_height + 2 * eps
          ],
          center = true);

      // screw_holes
      dupe_y(motor_depth - 2 * motor_screw_side_distance)
        dupe_z(motor_height - 2 * motor_screw_top_distance)
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + motor_mount_screw_clearance,
            h = bb_xdim(motor_mount_bb) + 2 * eps,
            center = true
          );
    }
  }
}

module motor_mount_sacrificial_bridging() {
  screw_effective_diameter = motor_screw_size
        + 2 * motor_mount_screw_clearance;
  L_bracket_clearance_x_offset =
      motor_cylinder_length
      + L_bracket_thickness
      + 2 * motor_mount_bracket_clearance;
  cylinder_hole_xmax =
      - bb_xdim(motor_mount_bb) / 2
      + motor_cylinder_length
      + motor_mount_cylinder_clearance;

  // at the end of the motor cylinder
  mov_x(cylinder_hole_xmax + layer_height / 2)
    cube([
      layer_height,
      L_bracket_inbetween_space
        - 2 * motor_mount_bracket_clearance,
      motor_shaft_diameter
        + 2 * motor_mount_shaft_clearance
        + 2 * eps
      ],
      center=true);

  //// end of L-bracket slide-in hole, for the screw holes
  mov_x(- bb_xdim(motor_mount_bb) / 2
        + L_bracket_clearance_x_offset
        + layer_height / 2)
    cube([
        layer_height,
        bb_ydim(motor_mount_bb),
        bb_zdim(motor_mount_bb)
      ],
      center = true);
}

module motor_mount_screws() {
  translate([
      bb_xmax(motor_mount_bb),
      bb_ycenter(motor_mount_bb),
      bb_zcenter(motor_mount_bb)
    ])
    dupe_y(motor_depth - 2 * motor_screw_side_distance)
    dupe_z(motor_height - 2 * motor_screw_top_distance)
    union() {
      color(washer_color)
        rot_y(90)
        M4_washer();
      color(screw_color)
        mov_x(M_screw_head_height(4) + M_washer_thickness(4))
        rot_y(-90)
        M4(20);
    }
}

module force_sensor() {
  // main chassey
  mov_y(sensor_right_bulge_thickness / 2)
  difference() {
    color(force_sensor_color)
    union() {
      // main body
      cube(bb_dim(sensor_inner_bb), center=true);
      // center bulge
      cube([
          bb_xdim(sensor_bb),
          sensor_center_bulge_depth,
          sensor_center_bulge_length
        ], center=true);
      // right bulge
      translate(- bb_dim(sensor_inner_bb) / 2
                - [0, sensor_right_bulge_thickness, 0])
        cube([
            bb_xdim(sensor_inner_bb),
            sensor_right_bulge_thickness + eps,
            bb_zdim(sensor_inner_bb) / 2
              - sensor_center_hole_offset
              - sensor_center_hole_diameter / 2
          ]);
    }

    // center holes
    dupe_z(2 * sensor_center_hole_offset)
      rot_x(-90)
      cylinder(d = sensor_center_hole_diameter,
               h = bb_ydim(sensor_bb) + 2 * eps,
               center = true);

    // bottom screw holes
    mov_z(- bb_zdim(sensor_inner_bb) / 2
           + sensor_bottom_screw_distance / 2
           + sensor_bottom_screw_offset_1)
      dupe_z(sensor_bottom_screw_distance)
      rot_y(90)
      cylinder(d = sensor_bottom_screw_size,
               h = bb_xdim(sensor_bb) + 2 * eps,
               center = true);

    // top screw holes
    mov_z(bb_zdim(sensor_inner_bb) / 2
          - sensor_top_screw_distance / 2
          - sensor_top_screw_offset_1)
      dupe_z(sensor_top_screw_distance)
      rot_y(90)
      cylinder(d = sensor_top_screw_size,
               h = bb_xdim(sensor_bb) + 2 * eps,
               center = true);
  }
}

module sensor_mount() {
  // move from bracket part centered at the origin to full thing centered at
  // the origin
  translate(bb_center(sensor_mount_bracket_part_bb)
            - bb_center(sensor_mount_bb))
  color(printed_color_1)
  union() {
    difference() {
      // main body
      cube(bb_dim(sensor_mount_bracket_part_bb), center=true);

      // L-bracket slide holes
      dupe_y(bb_ydim(L_brackets_bb) - L_bracket_width)
        cube([
            L_bracket_thickness
              + 2 * sensor_mount_bracket_clearance,
            L_bracket_width
              + 2 * sensor_mount_bracket_clearance,
            sensor_mount_height
              + 2 * eps
          ], center=true);

      // screw holes
      dupe_y(bb_ydim(L_brackets_bb) - L_bracket_width)
        dupe_z(sensor_bottom_screw_distance)
        rot_y(90)
        cylinder(d = sensor_bottom_screw_size
                     + 2 * sensor_mount_screw_clearance,
                 h = bb_xdim(sensor_mount_bracket_part_bb) + 2 * eps,
                 center = true);

      // hex holes on one side for nuts to slide into
      mov_x(- bb_xdim(sensor_mount_bracket_part_bb) / 2
            + sensor_mount_nut_depth / 2
            - eps / 2)
        mov_y(L_bracket_width / 2
              + L_bracket_inbetween_space / 2)
        dupe_z(sensor_bottom_screw_distance)
        rot_y(90)
        rot_z(30)
        cylinder(r = M_nut_outer_radius(sensor_bottom_screw_size)
                     + sensor_mount_screw_clearance,
                 h = sensor_mount_nut_depth + eps,
                 $fn = 6,
                 center = true);
    }

    // part to wrap around the force sensor
    translate(- bb_center(sensor_mount_bracket_part_bb)
              + bb_center(sensor_mount_sensor_part_bb))
    difference() {
      mov_x(eps / 2)
        cube(bb_dim(sensor_mount_sensor_part_bb) + [eps, 0, 0],
             center = true);
      // space for the sensor
      mov_x(sensor_mount_sensor_wall / 2 + eps)
        cube([
            bb_xdim(sensor_inner_bb)
              + 2 * sensor_mount_sensor_clearance
              + 2 * eps,
            bb_ydim(sensor_bb)
              + 2 * sensor_mount_sensor_clearance,
            bb_zdim(sensor_mount_sensor_part_bb)
              + 2 * eps
          ], center = true);
      // slit at the end
      mov_x(- bb_xdim(sensor_mount_sensor_part_bb) / 2
            + sensor_mount_sensor_wall / 2)
        cube([
            sensor_mount_sensor_wall
              + 2 * eps,
            sensor_mount_sensor_wall_slit_width,
            bb_zdim(sensor_mount_sensor_part_bb)
              + 2 * eps
          ], center = true);
    }
  }
}

module sensor_mount_screws() {
  // screws mounting L to printed 2 to force sensor
  translate([
      bb_xmax(sensor_mount_bb),
      bb_ycenter(sensor_inner_bb),
      bb_zcenter(sensor_mount_bb)
    ])
    dupe_z(sensor_bottom_screw_distance)
    union() {
      color(screw_color)
        mov_x(M_screw_head_height(5)
              + M_washer_thickness(5))
        rot_y(-90)
        M5(20);
      color(washer_color)
        rot_y(90)
        M5_washer();
    }

  // screws on the other L brackets attached with nuts
  translate([
      bb_xmax(sensor_mount_bb),
      bb_ymax(L_brackets_bb)
        - L_bracket_width / 2,
      bb_zcenter(sensor_mount_bb)
    ])
    dupe_z(sensor_bottom_screw_distance)
    union() {
      color(screw_color)
        mov_x(M_screw_head_height(5)
              + M_washer_thickness(5))
        rot_y(-90)
        M5(16);
      color(washer_color)
        rot_y(90)
        M5_washer();
      color(nut_color)
        mov_x(- M_nut_height(5)
              - bb_xdim(sensor_mount_bracket_part_bb)
              + sensor_mount_nut_depth)
        rot_y(90)
        rot_z(30)
        M5_nut();
    }
}

module mounted_L_binders() {
  translate([
      platform_width / 2,
      platform_depth / 2,
      platform_height
        + L_bracket_thickness
    ])
    dupe_x((platform_width + L_binder_width) / 2)
    L_binder();
}

module L_binder() {
  L_binder_depth =
      2 * L_bracket_width
        + 2 * L_binder_bracket_clearance
        + L_bracket_inbetween_space
        + 2 * L_binder_wall_thickness;
  L_binder_height =
      2 * L_bracket_thickness
        + 2 * L_binder_bracket_clearance
        + 2 * L_binder_wall_thickness;
  color(printed_color_1)
  difference() {
    cube([
        L_binder_width,
        L_binder_depth,
        L_binder_height
      ], center=true);
    dupe_y(L_bracket_width/2
           + L_bracket_inbetween_space/2)
      cube([
          L_binder_width + 2*eps,
          L_bracket_width
            + 2 * L_binder_bracket_clearance,
          2 * L_bracket_thickness
            + 2 * L_binder_bracket_clearance
        ], center=true);
  }
}

module mounted_motor_coupler() {
  translate([
      bracket_x_mount
        + L_bracket_thickness
        + motor_mount_front_buffer
        + motor_coupler_mount_x_offset,
      platform_depth / 2,
      motor_z_mount
        + motor_height / 2
    ])
    rot_y(90)
    rot_z(180)
    motor_coupler();
}

module motor_coupler() {
  color(printed_color_1)
  difference() {
    // main body and prismatic joint
    union() {
      cylinder(d=motor_coupler_diameter, h=motor_coupler_length);
      mov_z(motor_coupler_length - eps)
        cylinder(d=motor_prismatic_outer_diameter,
                 h=motor_prismatic_length+eps,
                 $fn=6);
    }
    // delete motor shaft
    difference() {
      mov_z(-eps)
        cylinder(r=motor_shaft_diameter/2 + motor_coupler_shaft_clearance,
                 h=motor_coupler_length
                   - 2 * layer_height
                   + 2 * eps);
      mov_x(
          motor_shaft_cutout_height
            - motor_shaft_diameter / 2
            + motor_coupler_shaft_clearance)
      mov_z(- 2 * eps)
        cube_pcp(
            motor_shaft_diameter,
            motor_shaft_diameter,
            motor_coupler_length + 3 * eps
          );
    }
    // remove a slit on the side
    // slit only up to prismatic piece on the last two layers
    mov_y(motor_prismatic_outer_diameter/2 * cos(30))
      mov_z(-eps)
      cube_cpp(motor_coupler_slit_size,
               motor_coupler_diameter,
               motor_coupler_length + 2 * eps);
    mov_z(-eps)
      cube_cpp(motor_coupler_slit_size,
               motor_coupler_diameter,
               motor_coupler_length
                 - 2 * layer_height
                 + 2 * eps);

    clamp_screw_y =
        motor_shaft_diameter / 4
          + motor_coupler_shaft_clearance / 2
          + motor_coupler_diameter / 4;
    clamp_screw_z = motor_coupler_screw_offset;
    // create holes for the screw, head, and nut
    // screw shaft hole
    translate([0, clamp_screw_y, clamp_screw_z])
      rot_y(90)
      cylinder(r=motor_coupler_screw_size/2 + motor_coupler_screw_clearance,
               h=motor_coupler_diameter,
               center=true);
    // screw head hole
    translate([
        motor_coupler_slit_size / 2
          + motor_coupler_clamp_thickness,
        clamp_screw_y,
        clamp_screw_z
      ])
      rot_y(90)
      cylinder(d=M_screw_head_diameter(motor_coupler_screw_size)
                 + 2 * motor_coupler_screw_clearance,
               h=motor_coupler_diameter);
    // nut hole
    translate([
        - motor_coupler_slit_size / 2
          - motor_coupler_clamp_thickness,
        clamp_screw_y,
        clamp_screw_z
      ])
      rot_y(-90)
      rot_z(30)
      cylinder(r=M_nut_outer_radius(motor_coupler_screw_size)
                 + motor_coupler_screw_clearance,
               h=motor_coupler_diameter,
               $fn=6);

    // prismatic inner shaft
    mov_z(motor_coupler_length
        + motor_prismatic_length
        - motor_prismatic_cavity_depth
        )
      cylinder(d=motor_prismatic_inner_diameter,
               h=max(motor_prismatic_length, motor_prismatic_cavity_depth + eps),
               $fn=6);
  }
}

module bearing() {
  translate([
      L_bracket_length
        + bracket_x_mount
        - bearing_thickness
        - sensor_mount_width_buffer
        - sensor_mount_bracket_clearance
        - 2 * sensor_mount_sensor_clearance
        - sensor_width
        - sensor_mount_sensor_wall
        - bearing_mount_bearing_sensor_x_buffer
        ,
      platform_depth / 2,
      motor_z_mount
        + motor_height / 2
    ])
    color(bearing_color)
    rot_y(90)
    difference() {
      cylinder(d=bearing_outer_diameter, h=bearing_thickness);
      mov_z(-eps)
        cylinder(d=bearing_inner_diameter, h=bearing_thickness + 2*eps);
    }
}

module mounted_bearing_mount() {
  translate([
      L_bracket_length
        + bracket_x_mount
        - bearing_thickness
        - sensor_mount_width_buffer
        - sensor_mount_bracket_clearance
        - 2 * sensor_mount_sensor_clearance
        - sensor_width
        - sensor_mount_sensor_wall
        - bearing_mount_bearing_sensor_x_buffer
        - bearing_mount_bearing_clearance
        ,
      platform_depth / 2,
      motor_z_mount
        + motor_height / 2
    ])
    bearing_mount();
}

module bearing_mount() {
  // hat for the force sensor
  hat_width =
        sensor_width
          + 2 * bearing_mount_sensor_clearance
          + 2 * bearing_mount_sensor_wall;
  hat_depth =
        sensor_depth
          + 2 * bearing_mount_sensor_clearance
          + 2 * bearing_mount_sensor_wall;
  hat_height =
        sensor_top_screw_offset_2
          + sensor_top_screw_size / 2
          + 2 * bearing_mount_sensor_wall
          + bearing_mount_sensor_clearance;
  head_top =
        - motor_height / 2
        - motor_z_mount
        + sensor_z_mount
        + sensor_height;
  hat_x =
        - hat_width / 2
        + bearing_thickness
        + bearing_mount_bearing_sensor_x_buffer
        + sensor_mount_sensor_wall
        + sensor_mount_sensor_clearance
        + sensor_width / 2
        + bearing_mount_bearing_clearance;
  hat_y =
        - sensor_depth / 2
        - bearing_mount_sensor_clearance
        - bearing_mount_sensor_wall
        - L_bracket_inbetween_space / 2
        - L_bracket_width / 2;
  hat_z =
        - hat_height
        + head_top
        + bearing_mount_sensor_wall
        + bearing_mount_sensor_clearance;

  bearing_cushion_diameter =
        bearing_outer_diameter
          + 2 * bearing_mount_bearing_clearance
          + 2 * bearing_mount_bearing_wall_thickness;

  color(printed_color_1)
  difference() {
    union() {
      // sensor hat
      translate([
          hat_x,
          hat_y,
          hat_z
        ])
        difference() {
          cube([
              hat_width,
              hat_depth,
              hat_height
            ]);
          // sensor cutout
          translate([
              bearing_mount_sensor_wall,
              bearing_mount_sensor_wall,
              -eps
            ])
            cube([
                sensor_width
                  + 2 * bearing_mount_sensor_clearance,
                sensor_depth
                  + 2 * bearing_mount_sensor_clearance,
                hat_height
                  - bearing_mount_sensor_wall
                  + eps
              ]);
          // cut off front half
          translate([
              2 * hat_width / 3,
              - eps,
              - eps
            ])
            cube([
                hat_width + eps,
                hat_depth + 2 * eps,
                hat_height + 2 * eps
              ]);
          // screw holes
          translate([
              hat_width / 2,
              hat_depth / 2,
              hat_height
                - bearing_mount_sensor_wall
                - bearing_mount_sensor_clearance
                - sensor_top_screw_offset_1
            ])
            rot_y(90)
            cylinder(d=bearing_mount_screw_size + 2*bearing_mount_screw_clearance,
                     h=hat_width + 2 * eps,
                     center=true);
          translate([
              hat_width / 2,
              hat_depth / 2,
              hat_height
                - bearing_mount_sensor_wall
                - bearing_mount_sensor_clearance
                - sensor_top_screw_offset_2
            ])
            rot_y(90)
            cylinder(d=bearing_mount_screw_size + 2*bearing_mount_screw_clearance,
                     h=hat_width + 2 * eps,
                     center=true);
        }

      // wall around bearing
      rot_y(90)
      translate([0, 0, - bearing_mount_bearing_wall_thickness])
      cylinder(d=bearing_cushion_diameter,
               h=bearing_thickness
                 + bearing_mount_bearing_wall_thickness);

      // connection between them
      intersection() {
        union() {
          rot_x(-60)
          translate([
              - bearing_mount_bearing_wall_thickness,
              - 100,
              - bearing_cushion_diameter / 2
            ])
            cube([100, 100, bearing_cushion_diameter]);
          rot_x(90 + 22)
          translate([
              - bearing_mount_bearing_wall_thickness,
              0,
              - bearing_cushion_diameter / 2
            ])
            cube([100, 100, bearing_cushion_diameter]);
        }
        //#cube([10, 10, 10], center=true);
      }
    }

    // subtract bearing
    rot_y(90)
      cylinder(d=bearing_outer_diameter + 2 * bearing_mount_bearing_clearance,
               h=bearing_thickness + eps);

    // subtract channel in bearing
    rot_y(90)
      mov_z(- bearing_mount_bearing_wall_thickness - eps)
      cylinder(d=bearing_inner_diameter + 2 * bearing_mount_center_hole_clearance,
               h=bearing_thickness
                 + bearing_mount_bearing_wall_thickness
                 + 2 * eps);
  }
}
