$fn = 16;
simplify = false;
profile = 4; // [0, 1, 2, 3, 4]

use <./helpers.scad>
use <./screwlib.scad>

mov_xy( 0,  0) M3(20, profile=profile, simplify=simplify);
mov_xy( 0, 10) M3(16, profile=profile, simplify=simplify);
mov_xy( 0, 20) M3(12, profile=profile, simplify=simplify);
mov_xy( 0, 30) M3( 8, profile=profile, simplify=simplify);
mov_xy(10,  0) M4(20, profile=profile, simplify=simplify);
mov_xy(10, 10) M4(16, profile=profile, simplify=simplify);
mov_xy(10, 20) M4(12, profile=profile, simplify=simplify);
mov_xy(10, 30) M4( 8, profile=profile, simplify=simplify);
mov_xy(20,  0) M5(20, profile=profile, simplify=simplify);
mov_xy(20, 10) M5(16, profile=profile, simplify=simplify);
mov_xy(20, 20) M5(12, profile=profile, simplify=simplify);
mov_xy(20, 30) M5( 8, profile=profile, simplify=simplify);
mov_xy(30,  0) M6(20, profile=profile, simplify=simplify);
mov_xy(30, 10) M6(16, profile=profile, simplify=simplify);
mov_xy(30, 20) M6(12, profile=profile, simplify=simplify);
mov_xy(30, 30) M6( 8, profile=profile, simplify=simplify);
mov_xy( 0,-10) M3_nut(profile=profile, simplify=simplify);
mov_xy(10,-10) M4_nut(profile=profile, simplify=simplify);
mov_xy(20,-10) M5_nut(profile=profile, simplify=simplify);
mov_xy(30,-10) M6_nut(profile=profile, simplify=simplify);
mov_xy( 0,-20) M3_washer();
mov_xy(10,-20) M4_washer();
mov_xy(20,-20) M5_washer();
mov_xy(30,-20) M6_washer();
