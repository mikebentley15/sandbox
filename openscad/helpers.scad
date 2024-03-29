/******************************************************************************/
/* Handy Functions                                                            */
/******************************************************************************/

module sphere_shell(rin, rout) {
  difference() {
    sphere(r=rout);
    sphere(r=rin);
  }
}

module cylinder_pipe(rin, rout, h, center=false) {
  difference() {
    cylinder(r=rout, h=h, center=center);
    eps = 0.04;
    mov_z(-eps) cylinder(r=rin, h=h + 4*eps, center=center);
  }
}

// apply the # operator if a condition is true
module hash_if(cond) { if (cond) { #children(); } else { children(); } }
// apply the % operator if a condition is true
module mod_if(cond) { if (cond) { %children(); } else { children(); } }


// Aliases for translate
module mov_x(d)            { translate(v=[ d,  0,  0]) children(); }
module mov_y(d)            { translate(v=[ 0,  d,  0]) children(); }
module mov_z(d)            { translate(v=[ 0,  0,  d]) children(); }
module mov_xy(xd, yd)      { translate(v=[xd, yd,  0]) children(); }
module mov_xz(xd, zd)      { translate(v=[xd,  0, zd]) children(); }
module mov_yz(yd, zd)      { translate(v=[ 0, yd, zd]) children(); }
module mov_xyz(xd, yd, zd) { translate(v=[xd, yd, zd]) children(); }

// Aliases for rotate
module rot_x(a)            { rotate(a=[ a,  0,  0]) children(); }
module rot_y(a)            { rotate(a=[ 0,  a,  0]) children(); }
module rot_z(a)            { rotate(a=[ 0,  0,  a]) children(); }
module rot_xy(xa, ya)      { rotate(a=[xa, ya,  0]) children(); }
module rot_xz(xa, za)      { rotate(a=[xa,  0, za]) children(); }
module rot_yz(ya, za)      { rotate(a=[ 0, ya, za]) children(); }
module rot_xyz(xa, ya, za) { rotate(a=[xa, ya, za]) children(); }

// Duplicate an object moving one positive and one negative
module dupe(dx=0, dy=0, dz=0) {
  displacement = [dx, dy, dz] / 2;
  translate(-displacement) children();
  translate( displacement) children();
}
// convenience methods for one dimension
module dupe_x(d) { dupe(dx=d) children(); }
module dupe_y(d) { dupe(dy=d) children(); }
module dupe_z(d) { dupe(dz=d) children(); }

// Mirror while duplicating
module mir(axis=[1, 1, 1], dx=0, dy=0, dz=0) {
  translate(-[dx/2, dy/2, dz/2]) children();
  translate([dx/2, dy/2, dz/2]) mirror(v=[1, 0, 0]) children();
}
module mir_x(dx=0, dy=0, dz=0) { mir(axis=[1, 0, 0], dx=dx, dy=dy, dz=dz) children(); }
module mir_y(dx=0, dy=0, dz=0) { mir(axis=[0, 1, 0], dx=dx, dy=dy, dz=dz) children(); }
module mir_z(dx=0, dy=0, dz=0) { mir(axis=[0, 0, 1], dx=dx, dy=dy, dz=dz) children(); }

// Move and mirror while duplicating
module mov_mir_x(d) { mir_x() mov_x(d) children(); }
module mov_mir_y(d) { mir_y() mov_y(d) children(); }
module mov_mir_z(d) { mir_z() mov_z(d) children(); }

// Rotate while duplicating
module rot_dupe_x(a, i) { for (j=[1:i]) { rot_x(a*j) children(); } }
module rot_dupe_y(a, i) { for (j=[1:i]) { rot_y(a*j) children(); } }
module rot_dupe_z(a, i) { for (j=[1:i]) { rot_z(a*j) children(); } }

// stretch with the given displacement
module stretch(displacement)
  { hull() { children(); translate(displacement) children(); } }
module stretch_x(dx) { stretch([dx, 0, 0]) children(); }
module stretch_y(dy) { stretch([0, dy, 0]) children(); }
module stretch_z(dz) { stretch([0, 0, dz]) children(); }

// cube_[pcn][pcn][pcn] is aligned [p]ositive [c]enter or [n]egative for x y z
module cube_ppp(x, y, z) { mov_xyz(   0,    0,    0) cube(size=[x, y, z]); }
module cube_ppc(x, y, z) { mov_xyz(   0,    0, -z/2) cube(size=[x, y, z]); }
module cube_ppn(x, y, z) { mov_xyz(   0,    0,   -z) cube(size=[x, y, z]); }

module cube_pcp(x, y, z) { mov_xyz(   0, -y/2,    0) cube(size=[x, y, z]); }
module cube_pcc(x, y, z) { mov_xyz(   0, -y/2, -z/2) cube(size=[x, y, z]); }
module cube_pcn(x, y, z) { mov_xyz(   0, -y/2,   -z) cube(size=[x, y, z]); }

module cube_pnp(x, y, z) { mov_xyz(   0,   -y,    0) cube(size=[x, y, z]); }
module cube_pnc(x, y, z) { mov_xyz(   0,   -y, -z/2) cube(size=[x, y, z]); }
module cube_pnn(x, y, z) { mov_xyz(   0,   -y,   -z) cube(size=[x, y, z]); }


module cube_cpp(x, y, z) { mov_xyz(-x/2,    0,    0) cube(size=[x, y, z]); }
module cube_cpc(x, y, z) { mov_xyz(-x/2,    0, -z/2) cube(size=[x, y, z]); }
module cube_cpn(x, y, z) { mov_xyz(-x/2,    0,   -z) cube(size=[x, y, z]); }

module cube_ccp(x, y, z) { mov_xyz(-x/2, -y/2,    0) cube(size=[x, y, z]); }
module cube_ccc(x, y, z) { mov_xyz(-x/2, -y/2, -z/2) cube(size=[x, y, z]); }
module cube_ccn(x, y, z) { mov_xyz(-x/2, -y/2,   -z) cube(size=[x, y, z]); }

module cube_cnp(x, y, z) { mov_xyz(-x/2,   -y,    0) cube(size=[x, y, z]); }
module cube_cnc(x, y, z) { mov_xyz(-x/2,   -y, -z/2) cube(size=[x, y, z]); }
module cube_cnn(x, y, z) { mov_xyz(-x/2,   -y,   -z) cube(size=[x, y, z]); }


module cube_npp(x, y, z) { mov_xyz(  -x,    0,    0) cube(size=[x, y, z]); }
module cube_npc(x, y, z) { mov_xyz(  -x,    0, -z/2) cube(size=[x, y, z]); }
module cube_npn(x, y, z) { mov_xyz(  -x,    0,   -z) cube(size=[x, y, z]); }

module cube_ncp(x, y, z) { mov_xyz(  -x, -y/2,    0) cube(size=[x, y, z]); }
module cube_ncc(x, y, z) { mov_xyz(  -x, -y/2, -z/2) cube(size=[x, y, z]); }
module cube_ncn(x, y, z) { mov_xyz(  -x, -y/2,   -z) cube(size=[x, y, z]); }

module cube_nnp(x, y, z) { mov_xyz(  -x,   -y,    0) cube(size=[x, y, z]); }
module cube_nnc(x, y, z) { mov_xyz(  -x,   -y, -z/2) cube(size=[x, y, z]); }
module cube_nnn(x, y, z) { mov_xyz(  -x,   -y,   -z) cube(size=[x, y, z]); }

// square_[pcn][pcn] is aligned [p]ositive [c]enter or [n]egative for x y
module square_pp(x, y) { mov_xy(   0,    0) square(size=[x, y]); }
module square_pc(x, y) { mov_xy(   0, -y/2) square(size=[x, y]); }
module square_pn(x, y) { mov_xy(   0,   -y) square(size=[x, y]); }

module square_cp(x, y) { mov_xy(-x/2,    0) square(size=[x, y]); }
module square_cc(x, y) { mov_xy(-x/2, -y/2) square(size=[x, y]); }
module square_cn(x, y) { mov_xy(-x/2,   -y) square(size=[x, y]); }

module square_np(x, y) { mov_xy(  -x,    0) square(size=[x, y]); }
module square_nc(x, y) { mov_xy(  -x, -y/2) square(size=[x, y]); }
module square_nn(x, y) { mov_xy(  -x,   -y) square(size=[x, y]); }

// bb is bounding box as [center, dimensions] where
// - center: 3-vector for the center of the bounding box
// - dimensions: 3-vector for the size of the box in each direction

// create a bounding box
function bb(center=[0,0,0], dim=[0,0,0]) = [center, dim];
module show_bb(bb) { %translate(bb_center(bb)) cube(bb_dim(bb), center=true); }

function bb_xmin(bb) = bb[0].x - bb[1].x/2;
function bb_ymin(bb) = bb[0].y - bb[1].y/2;
function bb_zmin(bb) = bb[0].z - bb[1].z/2;
function bb_min(bb)  = [bb_xmin(bb), bb_ymin(bb), bb_zmin(bb)];

function bb_xmax(bb) = bb[0].x + bb[1].x/2;
function bb_ymax(bb) = bb[0].y + bb[1].y/2;
function bb_zmax(bb) = bb[0].z + bb[1].z/2;
function bb_max(bb)  = [bb_xmax(bb), bb_ymax(bb), bb_zmax(bb)];

function bb_dim(bb) = bb[1];
function bb_xdim(bb) = bb[1].x;
function bb_ydim(bb) = bb[1].y;
function bb_zdim(bb) = bb[1].z;

function bb_center(bb) = bb[0];
function bb_xcenter(bb) = bb[0].x;
function bb_ycenter(bb) = bb[0].y;
function bb_zcenter(bb) = bb[0].z;

function bb_join(bb1, bb2) = bb(
    center = [
      (max(bb_xmax(bb1), bb_xmax(bb2)) + min(bb_xmin(bb1), bb_xmin(bb2))) / 2,
      (max(bb_ymax(bb1), bb_ymax(bb2)) + min(bb_ymin(bb1), bb_ymin(bb2))) / 2,
      (max(bb_zmax(bb1), bb_zmax(bb2)) + min(bb_zmin(bb1), bb_zmin(bb2))) / 2
    ],
    dim = [
      max(bb_xmax(bb1), bb_xmax(bb2)) - min(bb_xmin(bb1), bb_xmin(bb2)),
      max(bb_ymax(bb1), bb_ymax(bb2)) - min(bb_ymin(bb1), bb_ymin(bb2)),
      max(bb_zmax(bb1), bb_zmax(bb2)) - min(bb_zmin(bb1), bb_zmin(bb2))
    ]
  );

// Used to fix render bugs on preview by making holes slightly longer than needed
eps = 0.04;
module nudge_x() { mov_x(-eps) children(); }
module nudge_y() { mov_y(-eps) children(); }
module nudge_z() { mov_z(-eps) children(); }

module poke_x() { mov_x(eps) children(); }
module poke_y() { mov_y(eps) children(); }
module poke_z() { mov_z(eps) children(); }

// see
// https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/undersized_circular_objects
// extracting the fudge factor
function outer_radius(r, fn)    = r/cos(180/fn);
function center_radius(r, fn)   = r * (1 + 1/cos(180/fn))/2;
function outer_diameter(d, fn)  = outer_radius(d, fn);
function center_diameter(d, fn) = center_radius(d, fn);
module cylinder_outer(h, r, fn) {
  cylinder(h=h, r=outer_radius(r, fn), $fn=fn);
}
module cylinder_center(h, r, fn) {
  cylinder(h=h, r=center_radius(r, fn), $fn=fn);
}

// So that circles can be aligned to xy
function round8(n) = ceil(round(n)/8)*8;
function round4(n) = ceil(round(n)/4)*4;
function get_fn_d(d, s=10) = round4(PI*d/s);
function get_fn_r(r, s=10) = get_fn_d(2*r, s);

// given a 'dict` returns the value associated with the given key
// the dict is a list of the form [(key0, value0), (key1, value1), ... ]
function lookup(d, k) = d[search([k],[for(i=[0:2:len(d)-2])d[i]])[0]*2+1];

// concatenate two lists together
function cat(L1, L2) = [for(L=[L1, L2], a=L) a];

module spline(p0, p1, p2, p3, fn, debug=false) {
  x0 = p0[0]; y0 = p0[1];   x1 = p1[0]; y1 = p1[1];
  x2 = p2[0]; y2 = p2[1];   x3 = p3[0]; y3 = p3[1];

  start = [[x3, 0], [x0, 0]];

  ts  = [for(i=[0:fn]) i/fn];
  tms = [for(i=[0:fn]) 1.0 - ts[i]];

  as  = [for(i=[0:fn]) pow(tms[i], 3)];
  bs  = [for(i=[0:fn]) 3.0 * ts[i] * pow(tms[i], 2)];
  cs  = [for(i=[0:fn]) 3.0 * pow(ts[i], 2) * tms[i]];
  ds  = [for(i=[0:fn]) pow(ts[i], 3)];

  xs  = [for(i=[0:fn]) as[i] * x0 + bs[i] * x1 + cs[i] * x2 + ds[i] * x3];
  ys  = [for(i=[0:fn]) as[i] * y0 + bs[i] * y1 + cs[i] * y2 + ds[i] * y3];

  body = [for(i=[0:fn]) [xs[i], ys[i]]];

  points = cat(start, body);
  polygon(points=points);

  if (debug) {
    color("red")   mov_xy(x0, y0) cube_xy(2,2,2);
    color("green") mov_xy(x1, y1) cube_xy(2,2,2);
    color("green") mov_xy(x2, y2) cube_xy(2,2,2);
    color("red")   mov_xy(x3, y3) cube_xy(2,2,2);
  }
}

module stripes(stripe_x, space_x, stripe_y, stripe_z, stripes) {
  unit_width = stripe_width + space_width;
  total_width = unit_width*stripes - space_width;
  mov_y(-total_width/2) {
    linear_extrude(height=stripe_z) {
      for (i=[0:stripes-1]) {
        mov_x(i*unit_width)
          square_pc(stripe_width, total_y);
      }
    }
  }
}

module 2d_hex_grid(rows, cols, r, z, s) {
  half_rows = floor(rows / 2);
  odd_cols = cols % 2;
  even_cols = abs(odd_cols - 1);
  half_cols = floor(cols / 2) - even_cols;

  d_x = 2*r + s;
  d_y = 3*(r + s/2)/sqrt(3);


  linear_extrude(height=z) {
    for (y=[-half_rows:half_rows]) {
      y_odd = abs(y) % 2;
      lower = -(half_cols + even_cols*y_odd);
      upper = half_cols - odd_cols*y_odd;
      for (x=[lower:upper]) {
        mov_xy(y_odd* d_x/2 + x*d_x, y*d_y)
          rot_z(180/6)
          circle(r=outer_radius(r, 6), $fn=6);
      }
    }
  }
}

// Based on
// https://www.thingiverse.com/thing:370871
module fibonacci_sphere(r, n)
{
  golden_ratio = 1.618033988749894848204586834365638117720309179805762862135;
  golden_angle = 360 * golden_ratio;
  np = 2*n - 1;

  function lon(i) = golden_angle*i;
  function xy(z) = sqrt(1 - pow(z,2));
  function z(i,n) = 2*i / (2*n + 1);
  function pos(i, n) = [cos(lon(i)) * xy(z(i,n)),
                        sin(lon(i)) * xy(z(i,n)),
                        z(i,n)];

  hull()
    polyhedron(points=[for(i=[-n:(n - 2)]) r * pos(i,n)],
               faces=[for(i=[0:3:2*n]) [i%np, (i + 1)%np, (i + 2)%np]]);
}

// Converts from inches to mm
function mm(in) = (in*25.4);

// Splits the time variable
function split_time(low, high) = low + (high - low)*$t;

// r: radius
// N: number of points
// th_i: initial theta (defaults to 0 degrees)
function circle_pts(r, N, th_i=0) =
  [for (i = [0:N-1]) let(th=th_i + i*360/N)
    r * [cos(th), sin(th)]];

// takes two arrays of equal size and returns a combined array by alternating
// elements between a and b
function interleave_arrays(a, b) =
  [for (i = [0:len(a)-1]) each [a[i], b[i]]];

// element-wise operations
function elementwise_add (a, b) = a + b;
function elementwise_sub (a, b) = a - b;
function elementwise_mult(a, b) =
    len(a) == len(b) ? [for (i = [0:len(a)-1]) a[i] * b[i]] : undef;
function elementwise_div (a, b) =
    len(a) == len(b) ? [for (i = [0:len(a)-1]) a[i] / b[i]] : undef;
function elementwise_pow (a, b) =
    len(a) == len(b) ? [for (i = [0:len(a)-1]) pow(a[i], b[i])] : undef;
// shorter aliases
function ew_add (a, b) = elementwise_add (a, b);
function ew_sub (a, b) = elementwise_sub (a, b);
function ew_mult(a, b) = elementwise_mult(a, b);
function ew_div (a, b) = elementwise_div (a, b);
function ew_pow (a, b) = elementwise_pow (a, b);
// TODO: implement the following operations as element-wise:
// - abs
// - sign
// - sin
// - cos
// - tan
// - acos
// - asin
// - atan
// - atan2
// - floor
// - round
// - ceil
// - ln
// - log
// - sqrt
// - exp
// - min
// - max
// - norm
// - cross



// generates 2D points describing the polygon of a star
// first point is [ro, 0] (i.e., aligned with the +x-axis)
//
// N: number of points on the star (number of points returned is 2*N)
// ro: outer radius (default = 1)
// ri: inner radius (default = 0.5)
// inner_rot_offset: degrees of rotational offset of the inside points.
//     By default, the inside points will be exactly in between the outside
//     points, but you can relatively "rotate" those inside points by setting
//     this value.
function star_pts(N=5, ro=1, ri=0.5, inner_rot_offset=0) =
  interleave_arrays(
    circle_pts(ro, N),
    circle_pts(ri, N, th_i = inner_rot_offset + 180/N));

// creates a star polygon using star_pts()  (see star_pts() doc)
module star(N=5, ro=1, ri=0.5) { polygon(star_pts(N=N, ro=ro, ri=ri)); }
