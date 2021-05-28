/* [General Settings] */

// Which model to render
part = "all"; // ["all", "motor-mount", "sensor-mount", "L-bind", "bearing-mount", "motor-prismatic-coupler", "needle-prismatic-coupler"]


/* [Printed Motor Mount] */

motor_mount_front_buffer      = 4;
motor_mount_depth_buffer      = 5;
motor_mount_height            = 50;
motor_mount_shaft_clearance   = 1;
motor_mount_bracket_clearance = 0.3;
motor_mount_screw_clearance   = 0.3;


/* [Printed Sensor Mount] */

sensor_mount_height                 = 28;
sensor_mount_depth_buffer           =  5;
sensor_mount_width_buffer           =  4;
sensor_mount_bracket_clearance      =  0.3;
sensor_mount_screw_clearance        =  0.3;
sensor_mount_screw_size             =  5;
sensor_mount_nut_depth              =  2;
sensor_mount_sensor_clearance       =  0.4;
sensor_mount_sensor_wall            =  3;
sensor_mount_sensor_wall_slit_width =  8;


/* [Printed L-Binder] */

/* [Printed Bearing Mount] */

/* [Printed Motor Prismatic Coupler] */

/* [Printed Needle Prismatic Coupler] */


/* [Platform] */

platform_width  = 60;
platform_height = 32;
platform_depth  = 42;

platform_side_screw_side_distance = 10;
platform_side_screw_top_distance  = 6;

platform_top_screw_width_side_distance = 15;
platform_top_screw_depth_side_distance = 6;

platform_screw_size = 4;


/* [L Bracket] */

L_bracket_height    = 120;
L_bracket_length    = 98;
L_bracket_width     = 24;
L_bracket_thickness = 2.9;

L_bracket_slit_width         = 5.2;
L_bracket_slit_end_distance  = 12;
L_bracket_bottom_slit_length = 70;
L_bracket_side_slit_length   = 90;


/* [Motor] */

motor_length = 48;
motor_depth  = 42;
motor_height = 42;

motor_cylinder_length     = 2;
motor_cylinder_diameter   = 22;
motor_shaft_length        = 24;
motor_shaft_diameter      = 5;
motor_shaft_cutout_length = 15;
motor_shaft_cutout_height = 4.5;

motor_screw_size          = 4;
motor_screw_depth         = 4.5;
motor_screw_side_distance = 5.5;
motor_screw_top_distance  = 5.5;

motor_mount_x_buffer      = 20;
motor_z_offset      = 20;


/* [Force Sensor] */

sensor_height    = 80;
sensor_width     = 12.7;
sensor_depth     = 12.7;

sensor_center_hole_diameter = 10.7;
sensor_center_hole_offset   = 3.4;

sensor_center_bulge_thickness = 1;
sensor_center_bulge_length    = 23.5;
sensor_center_bulge_depth     = 12.7;

sensor_bottom_screw_size      = 5;
sensor_bottom_screw_offset_1  = 5;
sensor_bottom_screw_offset_2  = 20;
sensor_top_screw_size         = 4;
sensor_top_screw_offset_1     = 5;
sensor_top_screw_offset_2     = 20;


/* [Bearing] */

// TODO: measure and fill in
bearing_outer_diameter = 10;
bearing_inner_diameter =  5;
bearing_thickness      =  5;


/* [General Print Settings] */

sacrificial_bridging                = true;
first_layer_height                  = 0.2;
layer_height                        = 0.2;
extrusion_width                     = 0.45;

/* [Advanced Settings] */

washer_thickness  = 0.75;
screw_head_height = 2.5;
m4_nut_height     = 3.25;
m5_nut_height     = 3.75;

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
printed_color_1    = "#ff0000aa";


// implementation

use <./helpers.scad>
use <screwlib.scad>

bracket_depth_overhang =
    (L_bracket_width / 2)
      - platform_top_screw_depth_side_distance;

bracket_x_mount =
    platform_top_screw_width_side_distance
      + L_bracket_slit_end_distance
      + L_bracket_bottom_slit_length
      - platform_screw_size / 2
      - L_bracket_length;

motor_z_mount =
    platform_height
      + L_bracket_height
      - L_bracket_slit_end_distance
      - motor_screw_top_distance
      + motor_screw_size / 2
      - L_bracket_side_slit_length
      + motor_z_offset;

motor_mount_depth =
    motor_cylinder_length
      + L_bracket_thickness
      + motor_mount_front_buffer;

motor_mount_width =
    platform_depth
      + 2 * bracket_depth_overhang
      + 2 * motor_mount_depth_buffer;

L_bracket_inbetween_space =
    platform_depth
    - 2 * platform_top_screw_depth_side_distance
    - L_bracket_width;

L_bracket_right_slit_z_bottom =
    platform_height
      + L_bracket_thickness
      + L_bracket_height
      - L_bracket_slit_end_distance
      - L_bracket_side_slit_length;

sensor_mount_thickness =
    L_bracket_thickness
      + 2 * sensor_mount_width_buffer
      + 2 * sensor_mount_bracket_clearance;

sensor_mount_width =
    2 * L_bracket_width
      + 2 * sensor_mount_depth_buffer
      + 2 * sensor_mount_bracket_clearance
      + L_bracket_inbetween_space;

sensor_mount_screw_distance =
    sensor_bottom_screw_offset_2
      - sensor_bottom_screw_offset_1;


if (part == "all") {
  platform();
  mounted_L_brackets();
  mounted_motor();
  mounted_motor_mount();
  motor_mount_screws();
  mounted_sensor_mount();
  sensor_mount_screws();
  mounted_force_sensor();
}

if (part == "motor-mount") {
  motor_mount(sacrificial_bridging=sacrificial_bridging);
}

if (part == "sensor-mount") {
  sensor_mount(sacrificial_bridging=sacrificial_bridging);
}

if (part == "L-bind") {

}

if (part == "bearing-mount") {

}

if (part == "motor-prismatic-coupler") {

}

if (part == "needle-prismatic-coupler") {

}


module platform() {
  color(platform_color)
  difference() {
    cube([platform_width, platform_depth, platform_height]);
    translate([platform_side_screw_side_distance,
               -eps,
               platform_height - platform_side_screw_top_distance])
      rot_x(-90)
      cylinder(r=platform_screw_size/2, h=platform_depth+2*eps);
    translate([platform_width - platform_side_screw_side_distance,
               -eps,
               platform_height - platform_side_screw_top_distance])
      rot_x(-90)
      cylinder(r=platform_screw_size/2, h=platform_depth+2*eps);
    translate([platform_top_screw_width_side_distance,
               platform_top_screw_depth_side_distance,
               platform_height / 2])
      cylinder(r=platform_screw_size/2, h=platform_height/2+eps);
    translate([platform_width - platform_top_screw_width_side_distance,
               platform_top_screw_depth_side_distance,
               platform_height / 2])
      cylinder(r=platform_screw_size/2, h=platform_height/2+eps);
    translate([platform_top_screw_width_side_distance,
               platform_depth - platform_top_screw_depth_side_distance,
               platform_height / 2])
      cylinder(r=platform_screw_size/2, h=platform_height/2+eps);
    translate([platform_width - platform_top_screw_width_side_distance,
               platform_depth - platform_top_screw_depth_side_distance,
               platform_height / 2])
      cylinder(r=platform_screw_size/2, h=platform_height/2+eps);
  }
}

// both brackets mounted to the platform with screws
module mounted_L_brackets() {
  // brackets
  color(bracket_color)
  translate([
      bracket_x_mount,
      platform_top_screw_depth_side_distance
        - L_bracket_width / 2,
      platform_height
    ])
    L_bracket();
  color(bracket_color)
  translate([
      bracket_x_mount,
      platform_depth
        - platform_top_screw_depth_side_distance
        - L_bracket_width / 2,
      platform_height
    ])
    L_bracket();

  // second stack of brackets on top
  color(bracket_color)
  translate([
      L_bracket_length
        + bracket_x_mount
        + L_bracket_thickness,
      L_bracket_width / 2
        + platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness
    ])
    rot_z(180)
    L_bracket();
  color(bracket_color)
  translate([
      L_bracket_length
        + bracket_x_mount
        + L_bracket_thickness,
      L_bracket_width / 2
        + platform_depth
        - platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness
    ])
    rot_z(180)
    L_bracket();

  // screws and washers of the first bracket
  color(washer_color)
  translate([
      platform_top_screw_width_side_distance,
      platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness * 2
    ])
    M4_washer();
  color(screw_color)
  translate([
      platform_top_screw_width_side_distance,
      platform_top_screw_depth_side_distance,
      screw_head_height
        + platform_height
        + L_bracket_thickness * 2
        + washer_thickness
    ])
    rot_x(180)
    M4(12);
  color(washer_color)
  translate([
      platform_width
        - platform_top_screw_width_side_distance,
      platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness * 2
    ])
    M4_washer();
  color(screw_color)
  translate([
      platform_width
        - platform_top_screw_width_side_distance,
      platform_top_screw_depth_side_distance,
      screw_head_height
        + platform_height
        + L_bracket_thickness * 2
        + washer_thickness
    ])
    rot_x(180)
    M4(12);

  // screws and washers of the second bracket
  color(washer_color)
  translate([
      platform_top_screw_width_side_distance,
      platform_depth
        - platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness * 2
    ])
    M4_washer();
  color(screw_color)
  translate([
      platform_top_screw_width_side_distance,
      platform_depth
        - platform_top_screw_depth_side_distance,
      screw_head_height
        + platform_height
        + L_bracket_thickness * 2
        + washer_thickness
    ])
    rot_x(180)
    M4(12);
  color(washer_color)
  translate([
      platform_width
        - platform_top_screw_width_side_distance,
      platform_depth
        - platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness * 2
    ])
    M4_washer();
  color(screw_color)
  translate([
      platform_width
        - platform_top_screw_width_side_distance,
      platform_depth
        - platform_top_screw_depth_side_distance,
      screw_head_height
        + platform_height
        + L_bracket_thickness * 2
        + washer_thickness
    ])
    rot_x(180)
    M4(12);

  // screws, washers, and nuts connecting the stacked brackets
  color(washer_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness * 2
    ])
    M4_washer();
  color(screw_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_top_screw_depth_side_distance,
      screw_head_height
        + platform_height
        + L_bracket_thickness * 2
        + washer_thickness
    ])
    rot_x(180)
    M4(12);
  color(nut_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_top_screw_depth_side_distance,
      platform_height
        - m4_nut_height
        - washer_thickness
    ])
    M4_nut();
  color(washer_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_top_screw_depth_side_distance,
      platform_height
        - washer_thickness
    ])
    M4_washer();
  color(washer_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_depth
        - platform_top_screw_depth_side_distance,
      platform_height
        + L_bracket_thickness * 2
    ])
    M4_washer();
  color(screw_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_depth
        - platform_top_screw_depth_side_distance,
      screw_head_height
        + platform_height
        + L_bracket_thickness * 2
        + washer_thickness
    ])
    rot_x(180)
    M4(12);
  color(nut_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_depth
        - platform_top_screw_depth_side_distance,
      platform_height
        - m4_nut_height
        - washer_thickness
    ])
    M4_nut();
  color(washer_color)
  translate([
      L_bracket_slit_end_distance
        + L_bracket_bottom_slit_length
        - 4,
      platform_depth
        - platform_top_screw_depth_side_distance,
      platform_height
        - washer_thickness
    ])
    M4_washer();
}

// single L-bracket near the origin
module L_bracket() {
  big_r = L_bracket_width / 2;
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

module mounted_force_sensor() {
  // TODO: mount it to the printed mounts
  // TODO: where do I want it?

  translate([
      L_bracket_length
        + bracket_x_mount
        - sensor_width
        - sensor_mount_width_buffer
        - sensor_mount_bracket_clearance,
      platform_top_screw_depth_side_distance
        - sensor_width / 2,
      L_bracket_right_slit_z_bottom
        - sensor_bottom_screw_offset_1
        + sensor_bottom_screw_size / 2
    ])
    force_sensor();
}

module force_sensor() {
  // main chassey
  difference() {
    color(force_sensor_color)
    union() {
      cube([sensor_width, sensor_depth, sensor_height]);
      translate([
        -sensor_center_bulge_thickness,
        0,
        (sensor_height - sensor_center_bulge_length) / 2
        ])
        cube([
            sensor_width + sensor_center_bulge_thickness*2,
            sensor_center_bulge_depth,
            sensor_center_bulge_length]);
    }

    // center holes
    translate([
        sensor_width / 2,
        -eps,
        sensor_height / 2 + sensor_center_hole_offset
      ])
      rot_x(-90)
      cylinder(r=sensor_center_hole_diameter/2, h=sensor_depth+2*eps);
    translate([
        sensor_width / 2,
        -eps,
        sensor_height / 2 - sensor_center_hole_offset
      ])
      rot_x(-90)
      cylinder(r=sensor_center_hole_diameter/2, h=sensor_depth+2*eps);

    // bottom screw holes
    translate([
        -eps,
        sensor_depth/2,
        sensor_bottom_screw_offset_1
      ])
      rot_y(90)
      cylinder(r=sensor_bottom_screw_size/2, h=sensor_width+2*eps);
    translate([
        -eps,
        sensor_depth/2,
        sensor_bottom_screw_offset_2
      ])
      rot_y(90)
      cylinder(r=sensor_bottom_screw_size/2, h=sensor_width+2*eps);

    // top screw holes
    translate([
        -eps,
        sensor_depth/2,
        sensor_height - sensor_top_screw_offset_1
      ])
      rot_y(90)
      cylinder(r=sensor_top_screw_size/2, h=sensor_width+2*eps);
    translate([
        -eps,
        sensor_depth/2,
        sensor_height - sensor_top_screw_offset_2
      ])
      rot_y(90)
      cylinder(r=sensor_top_screw_size/2, h=sensor_width+2*eps);
  }
}

module mounted_motor() {
  translate([
      -motor_length
        - motor_cylinder_length
        + bracket_x_mount,
      (platform_depth - motor_depth) / 2,
      motor_z_mount
    ])
    motor();
}

module motor() {
  color(motor_color)
  difference() {
    union() {
      // main body
      intersection() {
        cube([motor_length, motor_depth, motor_height]);
        translate([
            motor_length / 2,
            motor_depth / 2,
            motor_height / 2
          ])
          rot_x(45)
          cube([
              motor_length + 2 * eps,
              motor_depth + 10,
              motor_height + 10
            ], center=true);
      }

      // cylinder
      translate([
          motor_length - eps,
          motor_depth / 2,
          motor_height / 2
        ])
        rot_y(90)
        cylinder(r=motor_cylinder_diameter/2, h=motor_cylinder_length+eps);

      // shaft
      translate([
          motor_length,
          motor_depth / 2,
          motor_height / 2
        ])
        rot_y(90)
        difference() {
          cylinder(r=motor_shaft_diameter/2, h=motor_shaft_length);
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
    translate([
        motor_length - motor_screw_depth,
        motor_screw_side_distance,
        motor_screw_top_distance
      ])
      rot_y(90)
      cylinder(r=motor_screw_size/2, h=motor_screw_depth+eps);
    translate([
        motor_length - motor_screw_depth,
        motor_depth  - motor_screw_side_distance,
        motor_screw_top_distance
      ])
      rot_y(90)
      cylinder(r=motor_screw_size/2, h=motor_screw_depth+eps);
    translate([
        motor_length - motor_screw_depth,
        motor_screw_side_distance,
        motor_height - motor_screw_top_distance
      ])
      rot_y(90)
      cylinder(r=motor_screw_size/2, h=motor_screw_depth+eps);
    translate([
        motor_length - motor_screw_depth,
        motor_depth  - motor_screw_side_distance,
        motor_height - motor_screw_top_distance
      ])
      rot_y(90)
      cylinder(r=motor_screw_size/2, h=motor_screw_depth+eps);
  }
}

module mounted_motor_mount() {
  translate([
      bracket_x_mount
        - motor_cylinder_length,
      platform_depth / 2
        - motor_mount_width / 2,
      motor_z_mount
        - motor_mount_height / 2
        + motor_height / 2
    ])
    motor_mount(sacrificial_bridging=false);
}

module motor_mount(sacrificial_bridging=false) {
  color(printed_color_1)
  union() {
    if (sacrificial_bridging) { motor_mount_sacrificial_bridging(); }
    difference() {
      // main body
      cube([
          motor_mount_depth,
          motor_mount_width,
          motor_mount_height
        ]);

      // shaft hole
      translate([
          -eps,
          motor_mount_width / 2,
          motor_mount_height / 2
        ])
        rot_y(90)
        cylinder(
            r = motor_shaft_diameter / 2 + motor_mount_shaft_clearance,
            h = motor_mount_depth + 2 * eps
          );

      // motor cylinder slide in
      translate([
          -eps,
          motor_mount_width / 2,
          motor_mount_height / 2
        ])
        rot_y(90)
        cylinder(
            r = motor_cylinder_diameter / 2 + motor_mount_shaft_clearance,
            h = motor_cylinder_length + eps
          );

      // bracket slide-in holes
      translate([
          motor_cylinder_length
            - motor_mount_bracket_clearance,
          motor_mount_depth_buffer
            - motor_mount_bracket_clearance,
          - eps
        ])
        cube([
            L_bracket_thickness + 2 * motor_mount_bracket_clearance,
            L_bracket_width + 2 * motor_mount_bracket_clearance,
            motor_mount_height + 2 * eps
          ]);
      translate([
          motor_cylinder_length
            - motor_mount_bracket_clearance,
          motor_mount_width
            - motor_mount_depth_buffer
            - L_bracket_width
            - motor_mount_bracket_clearance,
          - eps
        ])
        cube([
            L_bracket_thickness + 2 * motor_mount_bracket_clearance,
            L_bracket_width + 2 * motor_mount_bracket_clearance,
            motor_mount_height + 2 * eps
          ]);

      // screw_holes
      translate([
          -eps,
          motor_mount_width / 2
            - motor_depth / 2
            + motor_screw_side_distance,
          motor_mount_height / 2
            - motor_height / 2
            + motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + motor_mount_screw_clearance,
            h = motor_mount_depth + 2 * eps
          );
      translate([
          -eps,
          motor_mount_width / 2
            + motor_depth / 2
            - motor_screw_side_distance,
          motor_mount_height / 2
            - motor_height / 2
            + motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + motor_mount_screw_clearance,
            h = motor_mount_depth + 2 * eps
          );
      translate([
          -eps,
          motor_mount_width / 2
            - motor_depth / 2
            + motor_screw_side_distance,
          motor_mount_height / 2
            + motor_height / 2
            - motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + motor_mount_screw_clearance,
            h = motor_mount_depth + 2 * eps
          );
      translate([
          -eps,
          motor_mount_width / 2
            + motor_depth / 2
            - motor_screw_side_distance,
          motor_mount_height / 2
            + motor_height / 2
            - motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + motor_mount_screw_clearance,
            h = motor_mount_depth + 2 * eps
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
      + motor_mount_bracket_clearance;

  // at the end of the motor cylinder
  translate([
    layer_height / 2
      + motor_cylinder_length,
    motor_mount_width / 2,
    motor_mount_height / 2
    ])
    cube([
      layer_height,
      L_bracket_inbetween_space
        - 2 * motor_mount_bracket_clearance,
      motor_shaft_diameter
        + 2 * motor_mount_shaft_clearance
        + 2 * eps
      ],
      center=true);

  // end of L-bracket slide-in hole, for the shaft hole
  translate([
    layer_height / 2 + L_bracket_clearance_x_offset,
    motor_mount_width / 2,
    motor_mount_height / 2
    ])
    cube([
      layer_height,
      motor_shaft_diameter
        + 2 * motor_mount_shaft_clearance
        + 2 * eps,
      motor_shaft_diameter
        + 2 * motor_mount_shaft_clearance
        + 2 * eps
      ],
      center=true);

  // end of L-bracket slide-in hole, for the screw holes
  translate([L_bracket_clearance_x_offset, 0, 0])
    cube([
      layer_height,
      motor_mount_width,
      motor_mount_height
      ]);
}

module motor_mount_screws() {
  //
  // screws mounting to brackets and motor
  //

  // bottom-left
  color(washer_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth,
      platform_depth / 2
        - motor_depth / 2
        + motor_screw_side_distance,
      motor_z_mount
        + motor_screw_top_distance
    ])
    rot_y(90)
    M4_washer();
  color(screw_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth
        + washer_thickness
        + screw_head_height,
      platform_depth / 2
        - motor_depth / 2
        + motor_screw_side_distance,
      motor_z_mount
        + motor_screw_top_distance
    ])
    rot_y(-90)
    M4(20);

  // bottom-right
  color(washer_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth,
      platform_depth / 2
        + motor_depth / 2
        - motor_screw_side_distance,
      motor_z_mount
        + motor_screw_top_distance
    ])
    rot_y(90)
    M4_washer();
  color(screw_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth
        + washer_thickness
        + screw_head_height,
      platform_depth / 2
        + motor_depth / 2
        - motor_screw_side_distance,
      motor_z_mount
        + motor_screw_top_distance
    ])
    rot_y(-90)
    M4(20);

  // top-left
  color(washer_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth,
      platform_depth / 2
        - motor_depth / 2
        + motor_screw_side_distance,
      motor_z_mount
        + motor_height
        - motor_screw_top_distance
    ])
    rot_y(90)
    M4_washer();
  color(screw_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth
        + washer_thickness
        + screw_head_height,
      platform_depth / 2
        - motor_depth / 2
        + motor_screw_side_distance,
      motor_z_mount
        + motor_height
        - motor_screw_top_distance
    ])
    rot_y(-90)
    M4(20);

  // top-right
  color(washer_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth,
      platform_depth / 2
        + motor_depth / 2
        - motor_screw_side_distance,
      motor_z_mount
        + motor_height
        - motor_screw_top_distance
    ])
    rot_y(90)
    M4_washer();
  color(screw_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + motor_mount_depth
        + washer_thickness
        + screw_head_height,
      platform_depth / 2
        + motor_depth / 2
        - motor_screw_side_distance,
      motor_z_mount
        + motor_height
        - motor_screw_top_distance
    ])
    rot_y(-90)
    M4(20);
}

module mounted_sensor_mount() {
  translate([
      L_bracket_length
        + bracket_x_mount,
      platform_top_screw_depth_side_distance
        - L_bracket_width / 2,
      // line up the bottom screw hole with the bottom of the L-bracket slit
      L_bracket_right_slit_z_bottom
        - sensor_mount_height / 2
        + sensor_mount_screw_distance / 2
        + sensor_mount_screw_size / 2
        + sensor_mount_screw_clearance
    ])
    sensor_mount();
}

module sensor_mount(sacrificial_bridging=false) {
  color(printed_color_1)
  union() {
    difference() {
      // main body
      translate([
          - sensor_mount_width_buffer
            - sensor_mount_bracket_clearance,
          - sensor_mount_depth_buffer
            - sensor_mount_bracket_clearance,
          0
        ])
        cube([
            sensor_mount_thickness,
            sensor_mount_width,
            sensor_mount_height
          ]);

      // L-bracket slide holes
      translate([
          - sensor_mount_bracket_clearance,
          - sensor_mount_bracket_clearance,
          - eps])
        cube([
            L_bracket_thickness
              + 2 * sensor_mount_bracket_clearance,
            L_bracket_width
              + 2 * sensor_mount_bracket_clearance,
            sensor_mount_height
              + 2 * eps
          ]);
      translate([
          - sensor_mount_bracket_clearance,
          L_bracket_width
            + L_bracket_inbetween_space
            - sensor_mount_bracket_clearance,
          -eps
        ])
        cube([
            L_bracket_thickness
              + 2 * sensor_mount_bracket_clearance,
            L_bracket_width
              + 2 * sensor_mount_bracket_clearance,
            sensor_mount_height
              + 2 * eps
          ]);

      // screw holes
      translate([
          L_bracket_thickness / 2,
          L_bracket_width / 2,
          sensor_mount_height / 2
            + sensor_mount_screw_distance / 2
        ])
        rot_y(90)
          cylinder(
              h=sensor_mount_thickness + 2*eps,
              r=sensor_mount_screw_size/2
                + sensor_mount_screw_clearance,
              center=true
            );
        //sensor_mount_screw_hole_and_nut();
      translate([
          L_bracket_thickness / 2,
          L_bracket_width / 2,
          sensor_mount_height / 2
            - sensor_mount_screw_distance / 2
        ])
        rot_y(90)
          cylinder(
              h=sensor_mount_thickness + 2*eps,
              r=sensor_mount_screw_size/2
                + sensor_mount_screw_clearance,
              center=true
            );
        //sensor_mount_screw_hole_and_nut();
      translate([
          L_bracket_thickness / 2,
          3 * L_bracket_width / 2
            + sensor_mount_bracket_clearance
            + L_bracket_inbetween_space,
          sensor_mount_height / 2
            + sensor_mount_screw_distance / 2
        ])
        sensor_mount_screw_hole_and_nut();
      translate([
          L_bracket_thickness / 2,
          3 * L_bracket_width / 2
            + sensor_mount_bracket_clearance
            + L_bracket_inbetween_space,
          sensor_mount_height / 2
            - sensor_mount_screw_distance / 2
        ])
        sensor_mount_screw_hole_and_nut();
    }
    // part to wrap around the force sensor
    sensor_wrap_depth =
          sensor_depth
            + 2 * sensor_mount_sensor_wall
            + 2 * sensor_mount_sensor_clearance;
    sensor_wrap_width =
          sensor_width
            + sensor_mount_sensor_wall
            + sensor_mount_sensor_clearance;
    translate([
      - sensor_mount_bracket_clearance
        - sensor_mount_width_buffer
        - sensor_wrap_width,
      L_bracket_width / 2
        - sensor_wrap_depth / 2,
      0
      ])
      difference() {
        cube([
            sensor_wrap_width
              + eps,
            sensor_wrap_depth,
            sensor_mount_height
          ]);
        translate([
            sensor_mount_sensor_wall,
            sensor_mount_sensor_wall,
            - eps
          ])
          cube([
              sensor_width
                + sensor_mount_sensor_clearance
                + 2 * eps,
              sensor_depth
                + 2 * sensor_mount_sensor_clearance,
              sensor_mount_height
                + 2 * eps
            ]);
        translate([
            - eps,
            - sensor_mount_sensor_wall_slit_width / 2
              + sensor_wrap_depth / 2,
            - eps
          ])
          cube([
              sensor_mount_sensor_wall
                + 2 * eps,
              sensor_mount_sensor_wall_slit_width,
              sensor_mount_height
                + 2 * eps
            ]);
      }
  }
}

module sensor_mount_screw_hole_and_nut() {
  m5_nut_radius = 4 / cos(30);
  union() {
    rot_y(90)
      cylinder(
          h=sensor_mount_thickness + 2*eps,
          r=sensor_mount_screw_size/2
            + sensor_mount_screw_clearance,
          center=true
        );
    translate([
        - sensor_mount_thickness / 2 - eps, 0, 0
      ])
      rot_y(90)
      rot_z(30)
      cylinder(
          h=sensor_mount_nut_depth + eps,
          r=m5_nut_radius + sensor_mount_screw_clearance,
          $fn=6
        );
  }
}

module sensor_mount_screws() {
  // screws mounting L to printed 2 to force sensor
  translate([
      L_bracket_length
        + bracket_x_mount
        + L_bracket_thickness
        + sensor_mount_bracket_clearance
        + sensor_mount_width_buffer,
      platform_top_screw_depth_side_distance,
      L_bracket_right_slit_z_bottom
        + sensor_mount_screw_distance / 2
        + sensor_mount_screw_size / 2
    ])
    dupe_z(sensor_mount_screw_distance / 2)
    union() {
      color(screw_color)
        mov_x(
            screw_head_height
              + washer_thickness
          )
        rot_y(-90)
        M5(20);
      color(washer_color)
        rot_y(90)
        M5_washer();
    }

  translate([
      L_bracket_length
        + bracket_x_mount
        + L_bracket_thickness
        + sensor_mount_bracket_clearance
        + sensor_mount_width_buffer,
      L_bracket_width
        + L_bracket_inbetween_space
        + platform_top_screw_depth_side_distance,
      L_bracket_right_slit_z_bottom
        + sensor_mount_screw_distance / 2
        + sensor_mount_screw_size / 2
    ])
    dupe_z(sensor_mount_screw_distance / 2)
    union() {
      color(screw_color)
        mov_x(
            screw_head_height
              + washer_thickness
          )
        rot_y(-90)
        M5(16);
      color(washer_color)
        rot_y(90)
        M5_washer();
      color(nut_color)
        mov_x(
            - m5_nut_height
              - sensor_mount_thickness
              + sensor_mount_nut_depth
          )
        rot_y(90)
        rot_z(30)
        M5_nut();
    }
}
