/* [General Settings] */

// Which model to render
part = "all"; // ["all", "base", "force-sensor", "motor", "3dprinted-bracket-1", "3dprinted-bracket-2"]


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
motor_mount_z_offset      = 20;


/* [Printed Bracket 1] */

printed_bracket_1_front_buffer      = 4;
printed_bracket_1_depth_buffer      = 10;
printed_bracket_1_height            = 50;
printed_bracket_1_shaft_clearance   = 1;
printed_bracket_1_bracket_clearance = 0.3;
printed_bracket_1_screw_clearance   = 0.3;


/* [General Print Settings] */

sacrificial_bridging                = true;
first_layer_height                  = 0.2;
layer_height                        = 0.2;
extrusion_width                     = 0.45;

/* [Advanced Settings] */

washer_thickness  = 0.75;
screw_head_height = 2.5;
m4_nut_height     = 3.25;

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
      + motor_mount_z_offset;

printed_bracket_1_depth =
    motor_cylinder_length
      + L_bracket_thickness
      + printed_bracket_1_front_buffer;

printed_bracket_1_width =
    platform_depth
      + bracket_depth_overhang * 2
      + printed_bracket_1_depth_buffer;

if (part == "all") {
  platform();
  mounted_L_brackets();
  mounted_force_sensor();
  mounted_motor();
  mounted_printed_bracket_1();
  printed_bracket_1_screws();
}

if (part == "3dprinted-bracket-1") {
  printed_bracket_1(sacrificial_bridging=sacrificial_bridging);
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
      20,
      7,
      40
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

module mounted_printed_bracket_1() {
  translate([
      bracket_x_mount
        - motor_cylinder_length,
      platform_depth / 2
        - printed_bracket_1_width / 2,
      motor_z_mount
        - printed_bracket_1_height / 2
        + motor_height / 2
    ])
    printed_bracket_1(sacrificial_bridging=false);
}

module printed_bracket_1(sacrificial_bridging=false) {
  color(printed_color_1)
  union() {
    if (sacrificial_bridging) { printed_bracket_1_sacrificial_bridging(); }
    difference() {
      // main body
      cube([
          printed_bracket_1_depth,
          printed_bracket_1_width,
          printed_bracket_1_height
        ]);

      // shaft hole
      translate([
          -eps,
          printed_bracket_1_width / 2,
          printed_bracket_1_height / 2
        ])
        rot_y(90)
        cylinder(
            r = motor_shaft_diameter / 2 + printed_bracket_1_shaft_clearance,
            h = printed_bracket_1_depth + 2 * eps
          );

      // motor cylinder slide in
      translate([
          -eps,
          printed_bracket_1_width / 2,
          printed_bracket_1_height / 2
        ])
        rot_y(90)
        cylinder(
            r = motor_cylinder_diameter / 2 + printed_bracket_1_shaft_clearance,
            h = motor_cylinder_length + eps
          );

      //// shaft slit
      //translate([
      //    -eps,
      //    printed_bracket_1_width / 2
      //      - motor_shaft_diameter / 2
      //      - printed_bracket_1_shaft_clearance,
      //    -eps
      //  ])
      //  cube([
      //      printed_bracket_1_depth + 2*eps,
      //      motor_shaft_diameter + 2 * printed_bracket_1_shaft_clearance,
      //      printed_bracket_1_height / 2 + eps
      //    ]);

      // bracket slide-in holes
      translate([
          motor_cylinder_length
            - printed_bracket_1_bracket_clearance,
          printed_bracket_1_depth_buffer / 2
            - printed_bracket_1_bracket_clearance,
          - eps
        ])
        cube([
            L_bracket_thickness + 2 * printed_bracket_1_bracket_clearance,
            L_bracket_width + 2 * printed_bracket_1_bracket_clearance,
            printed_bracket_1_height + 2 * eps
          ]);
      translate([
          motor_cylinder_length
            - printed_bracket_1_bracket_clearance,
          printed_bracket_1_width
            - printed_bracket_1_depth_buffer / 2
            - L_bracket_width
            - printed_bracket_1_bracket_clearance,
          - eps
        ])
        cube([
            L_bracket_thickness + 2 * printed_bracket_1_bracket_clearance,
            L_bracket_width + 2 * printed_bracket_1_bracket_clearance,
            printed_bracket_1_height + 2 * eps
          ]);

      // screw_holes
      translate([
          -eps,
          printed_bracket_1_width / 2
            - motor_depth / 2
            + motor_screw_side_distance,
          printed_bracket_1_height / 2
            - motor_height / 2
            + motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + printed_bracket_1_screw_clearance,
            h = printed_bracket_1_depth + 2 * eps
          );
      translate([
          -eps,
          printed_bracket_1_width / 2
            + motor_depth / 2
            - motor_screw_side_distance,
          printed_bracket_1_height / 2
            - motor_height / 2
            + motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + printed_bracket_1_screw_clearance,
            h = printed_bracket_1_depth + 2 * eps
          );
      translate([
          -eps,
          printed_bracket_1_width / 2
            - motor_depth / 2
            + motor_screw_side_distance,
          printed_bracket_1_height / 2
            + motor_height / 2
            - motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + printed_bracket_1_screw_clearance,
            h = printed_bracket_1_depth + 2 * eps
          );
      translate([
          -eps,
          printed_bracket_1_width / 2
            + motor_depth / 2
            - motor_screw_side_distance,
          printed_bracket_1_height / 2
            + motor_height / 2
            - motor_screw_top_distance
        ])
        rot_y(90)
        cylinder(
            r = motor_screw_size / 2
                + printed_bracket_1_screw_clearance,
            h = printed_bracket_1_depth + 2 * eps
          );
    }
  }
}

module printed_bracket_1_sacrificial_bridging() {
  L_bracket_inbetween_space =
      platform_depth
      - 2 * platform_top_screw_depth_side_distance
      - L_bracket_width;
  screw_effective_diameter = motor_screw_size
        + 2 * printed_bracket_1_screw_clearance;
  L_bracket_clearance_x_offset =
      motor_cylinder_length
      + L_bracket_thickness
      + printed_bracket_1_bracket_clearance;

  // at the end of the motor cylinder
  translate([
    layer_height / 2
      + motor_cylinder_length,
    printed_bracket_1_width / 2,
    printed_bracket_1_height / 2
    ])
    cube([
      layer_height,
      L_bracket_inbetween_space
        - 2 * printed_bracket_1_bracket_clearance,
      motor_shaft_diameter
        + 2 * printed_bracket_1_shaft_clearance
        + 2 * eps
      ],
      center=true);

  // end of L-bracket slide-in hole, for the shaft hole
  translate([
    layer_height / 2 + L_bracket_clearance_x_offset,
    printed_bracket_1_width / 2,
    printed_bracket_1_height / 2
    ])
    cube([
      layer_height,
      motor_shaft_diameter
        + 2 * printed_bracket_1_shaft_clearance
        + 2 * eps,
      motor_shaft_diameter
        + 2 * printed_bracket_1_shaft_clearance
        + 2 * eps
      ],
      center=true);

  // end of L-bracket slide-in hole, for the screw holes
  translate([L_bracket_clearance_x_offset, 0, 0])
    cube([
      layer_height,
      printed_bracket_1_width,
      printed_bracket_1_height
      ]);
}

module printed_bracket_1_screws() {
  //
  // screws mounting to brackets and motor
  //

  // bottom-left
  color(washer_color)
  translate([
      bracket_x_mount
        - motor_cylinder_length
        + printed_bracket_1_depth,
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
        + printed_bracket_1_depth
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
        + printed_bracket_1_depth,
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
        + printed_bracket_1_depth
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
        + printed_bracket_1_depth,
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
        + printed_bracket_1_depth
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
        + printed_bracket_1_depth,
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
        + printed_bracket_1_depth
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
