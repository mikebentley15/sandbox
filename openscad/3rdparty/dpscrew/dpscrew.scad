/*
 This is another implementation of a thread/screw library for Openscad.
 The screw() function creats either an ISO standard thread profile, or a
 nut profile to use in combination with the difference() operator.

 usage:
 screw(size = diameter, length = length, pitch = pitch, profile = [0/1/2/3],
     tolerance = percentage, lpt = 20, rf = 36)

 Parameters:

 size: diameter of the bolt or nut threads in mm..  For bolt profiles, this
 diameter is reduced to 50% of the ISO tolerance range for the given pitch, and
 for nuts it is increased to 50% of the tolerance range for nuts.  Note, that
 this is approximated.  See explaination below.

 length: length of the screw in mm.

 pitch: thread petch in mm.

 profile: Odd numberd profiles are for bolts, odd numbered are for nuts.
 Profiles 0 and 1 threads follow the ISO standard 60 degree fundamental
 trinagle, with the upper 1/8 flattened out for bolt threads, and 1/4 flattened
 out for nut threads.

 Profiles 2 and 3 are similar to 0 and 1, but the thread profile is more rounded
 out.  Since 3D printers print in layers at .1 to .15 mm thick, the threads come
 out jagged and tend to bind up, requiring more space between the bolt and nut.
 So use profile 2 for bolts, and 3 for nuts, if you want parts that actually
 screw together properly.  Use profiles 0 and 1 to create models that look more
 accurate on the screen.

 tolerance: Default is 50%.  This creates bolts and nuts sized in the middle of
 the tolerance range for the given diameter (see below).

 Examples:

 use <dpscrew.scad> 
 // M10x1.25 bolt, 20mm long.
 cylinder(d = 16, h = 3, $fn=6);
 translate([0, 0, 3])
    screw(size = 10, length = 20, pitch = 1.25, profile = 3);
   
 // Nut to go with this screw
 translate([30, 0, 0])
 difference() {
    cylinder(d = 16, h = 6, $fn=6);
    screw(size = 10, length = 6, pitch = 1.25, profile = 2);
 }

 Note on thread tolerances.  The height of the threads fundamental triangle (H)
 is 1/2 pitch * sqrt(3).  The usable section is 5/8 H. (See diagram of the ISO
 metric screw profile in the ISO standard or Wikipedia).  For bolts, this is
 set back from the diameter width by 5% to 35%, for an average of 20%.  For
 bolts, this is increased between 0% and 60%, for an average of 30%.  So in
 total there is an average of 50% of the usable section of the thread in
 contact.  Note, that in the actual ISO standard, these numbers vary slightly
 based on pitch size, and diameter.
 These numbers given are average, and usable over the range of bolts and nuts
 that would typically be used on a 3D printer.  If you want exact tolerances,
 this is available in a separate plug in library used in conjuction with this
 one.

*/

/* Thread profiles
 * These functions define the profile for the shape of an screw thread.
 * Odd numbered profiles are for bolts, and even numbered ones are for
 * nuts.
 */

// Profile of an ISO bolt thread
// increases lineraly from 0 to 157.5, then flattens out at 202.5,
// finally decreasing to 360.
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

// Distance from 180 degrees
function d180(d) = abs(180 - (abs(d) % 360));

//function range(a, b, c) = c < a ? a : c > b ? b : c;

threadprofiles = [
    [ 0, "ISO Nut", .25, .875 ],
    [ 1, "ISO Bolt", .25, .875],
    [ 2, "ISO Bolt", .25, .875 ],
    [ 3, "ISO Bolt", .25, .875 ]
];

// wrapper to call a given thread profile since first class functions
// aren't supported yet.
function threadprofile(n, d) =
    n == 0 ? threadprofile_0(d) :
    n == 1 ? threadprofile_1(d) :
    n == 2 ? threadprofile_2(d) :
    n == 3 ? threadprofile_3(d) :
    0;

/* Draws a circle(ish) or heart shape, similar to a cardioid, 
 * which represnts a single disk out of a stack
 * of disks that simulates a screw.
 * This is a circle with the radius growing from the inner radius of a screw
 * (the valley), to the till it hits the halfway mark (the peak), then
 * shrinking back down to the valley.
 * A stack of these, each rotated slightly, represents a screw.
 *
 * r1 = inside radius
 * r2 = outsice radius
 * profile = thread profile number
 * rotation = self explainatory
 * t = thickness
 * h = percentage of thread height (for Higbee cut)
 *     0-1 for internal (bolt) threads, 2-3 for external (nut) threads
 * rf = radial facets (number of sides to draw)
 */
module screwslice(r1, r2, profile = 0, rotation = 0, t = .1, h = 1, taper=0, rf = 36) {
    d = 360 / rf;
    p = h <= 1 ?
	[ for (i = [0:rf]) [
	    sin(d * i) * (r1 + 
		(threadprofile(profile, d * i + rotation) * h * (r2 - r1)
		)),
	    cos(d * i) * (r1 + 
		(threadprofile(profile, d * i + rotation) * h * (r2 - r1)
		))]] :
	[ for (i = [0:rf]) [
	    sin(d * i) * (r1 + (
		(1 - (1 - threadprofile(profile, d * i + rotation)) * 
		(h - 2)) *
		(r2 - r1)
		)),
	    cos(d * i) * (r1 + (
		(1 - (1 - threadprofile(profile, d * i + rotation)) * 
		(h - 2)) *
		(r2 - r1)
		))]];
    if (taper == 0)
	linear_extrude(height = t)
	    polygon(p);
    else if (taper > 0) 
	linear_extrude(height = t, scale=taper)
	    polygon(p);
    else if (taper < 0) 
	mirror([1,0,0])
	    rotate([0,180,0])
		linear_extrude(height = t, scale=taper * -1)
		    polygon(p);
}

/* Draws a screw or a threaded nut hole.
 * All sizes are in mm.
 * size = nominal diameter.  Auto-adjusted smaller for screws, or larger
     for nut threads.
 * length = length of screw.
 * pitch = thread pitch.
 * profile = thread profile.  0 = standard nut, 1 = standard bolt,
     2 = modified thread nut, 3 = modified thread bolt.  2 and 3 work better
     for 3d printing.
 * tolerance = 0 - 100.  Default 50.  Larger makes the screw/nut tighter,
     smaller is loser.
 * lpt = layers per thread.  Each layer is a circle(ish) shaped slice of
 *  the screw.  Defaults to 16.  Set to higer for prettier screws, lower for
    faster rendering.
 * rf = radial facets -- how many surfaces make up the circles that compose
 * the layers.  Defaults to 36.  Again: higher = prettier, lower = faster.
 * Dm = 1: Diameter represents Major diameter.  0: represents Minor diameter.
     Defaults to 1.  Mostly useful for other library functions.
 * SeqD = 0: Auto-adjust diameter based on profile.  1: use size for exact
     diameter.  Mostly useful for other library functions.
 */

module screw(size, length, pitch, profile, tolerance = 50, lpt = 16, rf = 36, Dm = 1, SeqD = 0) {
    // Height of the overall thread profile (including unused peak and valley)
    H = .5 * pitch * pow(3, .5);
    H3 = H * (5/8);
    // SeqD -- Size equals Diameter.  If set to 1, then use size as the
    // diameter.  Else auto-compute bolt/nut diameter.
    diameter = SeqD == 0 ? (profile % 2) == 1 ?
	size - (.30 * ((100 -tolerance) / 100) * H3) - (.05 * H3) :
	size + (.60 * ((100 - tolerance) / 100) * H3) : size;

    // Compute r1 and r2 based on if diameter represents Dmin (Dm = 0), or
    // Dmaj (Dm == 1).  Default is Dmaj.
    r1 = (diameter / 2) - H * threadprofiles[profile][(Dm + 2)];
    r2 = r1 + H;

    module drawslice(r1, r2, profile, rtn1, rtn2, thickness, h1 = 1, h2 = 1, rf=rf, fl=0) {
	hull() {
	    screwslice(r1, r2, profile, rtn1, .001, h1, .001, rf=rf);
	    translate([0, 0, thickness])
		screwslice(r1, r2, profile, rtn2, .001, h2, fl == 2 ? -.001 : .1, rf=rf);
	}
    }
    threads = length / pitch;
    layers = threads * lpt;

    for (i = [0:floor(layers) - 1]) {

	translate([0, 0, i * (pitch / lpt)])
	    drawslice(r1, r2, profile,
		(i % lpt) * (360 / lpt), ((i + 1) % lpt) * (360 / lpt),
		pitch / lpt,

		i < lpt ? i / lpt + (profile % 2 == 0 ? 2 : 0) :
		(i > floor(layers) -  lpt) ? (floor(layers) - i) / lpt +
		(profile % 2 == 0 ? 2 : 0) : 1,

		(i + 1 < lpt) ? (i + 1) / lpt + (profile % 2 == 0 ? 2 : 0) :
		(i + 1 > floor(layers) -  lpt) ? (floor(layers) - (i + 1)) / lpt +
		(profile % 2 == 0 ? 2 : 0) : 1, rf=rf, fl = (i == 0 ? 1 : i == layers - 1 ? 2 : 0));
    }
    if (floor(layers) < layers) {
	translate([0, 0,  floor(layers) * (pitch / lpt)])
	    drawslice(r1, r2, profile, 
		(floor(layers) % lpt) * (360 / lpt),
		(layers %  lpt) * (360 / lpt),
		length - floor(layers) * (pitch / lpt), profile % 2 == 0 ? 2 : 0,
		profile % 2 == 0 ? 2: 0, rf=rf, fl=2);
    }
}
