// center, box dimensions
obj_bb     = [[4, 4, 4], [2, 3, 4]];

difference() {
  obj();
  #zhole();
}

module obj() {
  translate(obj_bb[0]) cube(obj_bb[1], center=true);
}

module zhole() {
  translate(obj_bb[0]) cube(obj_bb[1] + [-.5, -.5, .5], center=true);
}

// bb is bounding box as [center, dimensions] where
// - center: 3-vector for the center of the bounding box
// - dimensions: 3-vector for the size of the box in each direction

function bb_xmin(bb) = bb[0].x - bb[1].x/2;
function bb_ymin(bb) = bb[0].y - bb[1].y/2;
function bb_zmin(bb) = bb[0].z - bb[1].z/2;

function bb_xmax(bb) = bb[0].x + bb[1].x/2;
function bb_ymax(bb) = bb[0].y + bb[1].y/2;
function bb_zmax(bb) = bb[0].z + bb[1].z/2;

function bb_dim(bb) = bb[1];
function bb_xdim(bb) = bb[1].x;
function bb_ydim(bb) = bb[1].y;
function bb_zdim(bb) = bb[1].z;

function bb_center(bb) = bb[0];
function bb_xcenter(bb) = bb[0].x;
function bb_ycenter(bb) = bb[0].y;
function bb_zcenter(bb) = bb[0].z;

