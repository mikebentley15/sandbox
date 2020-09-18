/* [Pan Settings] */

pan_radius = 164.388;
pan_thickness = 4;
pan_height = 240;
bevel_top_height = 8.9;
bevel_top_radius = 12.2;

handle_width = 79.4;
handle_height = 40.0;
handle_length = 49.5;
handle_bottom_thickness = 4.0;
handle_base_thickness = 11.7;

cavity_depth = 8.6;
cavity_width = 69.0;

hole_radius = 4.0;
hole_bottom_offset = 4.1;

front_hole_support_depth = 6.1;
front_hole_support_width = 18.3;

front_hole_cutout_height = 14.2;
front_hole_cutout_width = 13.0;
front_hole_cutout_depth = 5.5;

bottom_cutout_thickness = 2.0;
bottom_cutout_depth = 4.8;
bottom_cutout_width = 13.0;
bottom_cutout_center_offset = (20.75 + 7.75) / 2;

/* [Model Settings] */

// Extra length when making holes
eps = 0.04;

circle_faces = 400;


$fn = circle_faces;

use <./helpers.scad>

aqua = [0.2, 1, 0.7];
skyblue = [0, 1, 1];

//mov_x(-pan_radius) color(aqua) pan();
handle();


module pan() {
  effective_height = pan_height - bevel_top_height;
  mov_z(-effective_height + handle_height/2 - bevel_top_height)
  union() {
    mov_z(effective_height) bevel_top();
    linear_extrude(effective_height) pan_silluette();
    linear_extrude(pan_thickness) circle(pan_radius);
  }
}

module pan_silluette() {
  difference() {
    circle(pan_radius);
    circle(pan_radius - pan_thickness);
  }
}

module bevel_top() {
  difference() {
    cut_cone(pan_radius, pan_radius + bevel_top_radius*2, bevel_top_height*2);
    mov_z(-eps)
      cut_cone(pan_radius - pan_thickness, 
               pan_radius - pan_thickness + bevel_top_radius*2,
               bevel_top_height*2 + eps*2);
  }
}

module cut_cone(r1, r2, h) {
  linear_extrude(h, scale = r2 / r1) circle(r1);
}

module handle() {
  mov_x(-pan_radius) 
  difference() {
    main_handle_shape();
    handle_hole();
    bottom_cutouts();
    hole_front_cutout();
  } // end of difference
  hole_support();
} // end of module handle()

module hole_support() {
  thickness = 2.5;
  height = 17.2;
  width = handle_base_thickness - 1.4;
  yshift = (bottom_cutout_width + thickness) / 2;
  xshift = pan_radius - sqrt(pan_radius*pan_radius - yshift*yshift);

  translate([-xshift + width/2, yshift, (height - handle_height) / 2])
    cube([width, thickness, height], center=true);
  translate([-xshift + width/2, -yshift, (height - handle_height) / 2])
    cube([width, thickness, height], center=true);
}

module handle_hole() {
  mov_z(-handle_height / 2 + hole_bottom_offset + hole_radius)
    mov_x(pan_radius)
    rot_y(90)
    cylinder(r=hole_radius, h=handle_length);
} // end of module handle_hole()

module hole_front_cutout() {
  h = front_hole_cutout_height;
  w = front_hole_cutout_width;
  d = front_hole_cutout_depth;
  mov_xz(pan_radius + d/2 + 13.0, -handle_height/2 + h/2)
    cube([d + 2*eps, w, h + eps], center=true);
} // end of module hole_front_cutout()

module bottom_cutouts() {
  bottom_cutout();
  x = handle_width/2 - bottom_cutout_center_offset;
  theta = asin(x / pan_radius);
  rot_z( theta) bottom_cutout();
  rot_z(-theta) bottom_cutout();
} // end of module bottom_cutouts()

module bottom_cutout() {
  mov_z((-handle_height + handle_bottom_thickness + bottom_cutout_thickness) / 2)
  cube([2 * (pan_radius + bottom_cutout_depth),
        bottom_cutout_width,
        bottom_cutout_thickness + eps],
       center=true);
} // end of module bottom_cutout()

module main_handle_shape() {
  width = handle_width;
  length = handle_length;
  thickness = 3;
  support_width = 15;
  base_thickness = handle_base_thickness;

  intersection() {
    difference() {
      union() {
        cylinder(r=pan_radius + base_thickness, h=handle_height, center=true);
        handle_curve_r = 150;
        mov_z(- handle_curve_r + handle_height/2 + handle_curve_r
              - sqrt(handle_curve_r*handle_curve_r - base_thickness*base_thickness))
          handle_flap(handle_curve_r, thickness);

        x = length - base_thickness + thickness;
        y = handle_height - handle_curve_r - thickness
            + sqrt(handle_curve_r*handle_curve_r - length*length);
        c = sqrt(x*x + y*y);
        phi = 180 - 2 * atan(x / y);
        r_m = c / (2 * sin(phi / 2));

        intersection() {
          mov_z(- handle_curve_r + handle_height/2 + handle_curve_r
                - sqrt(handle_curve_r*handle_curve_r - base_thickness*base_thickness))
            taurus(handle_curve_r, pan_radius);
          union() {
            difference() {
              mov_x(pan_radius + length/2 + 1)
              union() {
                mov_y(width/2 - support_width/2)
                  cube([length, support_width, handle_height], center=true);
                mov_y(-width/2 + support_width/2)
                  cube([length, support_width, handle_height], center=true);
              } // end of union
              mov_z(-handle_height/2 + y - r_m)
                mov_x(pan_radius + length)
                rot_x(90)
                cylinder(r=r_m, h=200, center=true);
            } // end of difference
            minkowski() {
              minkowski_radius = 1.0;
              mov_x(pan_radius + handle_base_thickness - 0.2
                    + (front_hole_support_depth - 2.2) / 2)
                cube([front_hole_support_depth - 2*minkowski_radius + 2.2,
                      front_hole_support_width - 2*minkowski_radius,
                      handle_height], center=true);
              sphere(minkowski_radius);
            }
          } // end of union
        } // end of intersection

        // lip
        lip_radius = 2.5;
        lip_shift = 1;
        mov_z(y - handle_height/2 + lip_shift)
          taurus(lip_radius,
                 pan_radius + length
                 - sqrt(lip_radius*lip_radius - lip_shift*lip_shift));
        
      } // end of union
      cylinder(r=pan_radius, h=handle_height + 2, center=true);
      main_cavity();
      mov_z(handle_height/2 - bevel_top_height)
        cut_cone(pan_radius, pan_radius + bevel_top_radius*2, bevel_top_height*2);
    } // end of difference
    cylinder(r=pan_radius + length, h=handle_height + 2*eps, center=true);
    mov_x(pan_radius + 13/2)
      mov_y(150 - width/2)
      cylinder(r=150, h=handle_height, center=true);
    mov_x(pan_radius + 13/2)
      mov_y(width/2 - 150)
      cylinder(r=150, h=handle_height, center=true);
  } // end of intersection
} // end of module main_handle_shape()

module handle_flap(r, thickness) {
  difference() {
    taurus(r, pan_radius);
    taurus(r - thickness, pan_radius);
  } // end of difference
} // end of module handle_flap

module taurus(r1, r2) {
  rotate_extrude() mov_x(r2) circle(r=r1);
} // end of module taurus()

module main_cavity() {
  depth = cavity_depth;
  width = cavity_width;
  intersection() {
    mov_z(handle_bottom_thickness)
      cylinder(r=pan_radius + depth, h=handle_height, center=true);
    cube([3*pan_radius, width, handle_height + 2*eps], center=true);
  } // end of intersection
} // end of module main_cavity
