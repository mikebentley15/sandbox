use <./helpers.scad>
use <./screwlib.scad>

$fn = 16;

mov_xy( 0,  0) M3(20);
mov_xy( 0, 10) M3(16);
mov_xy( 0, 20) M3(12);
mov_xy( 0, 30) M3( 8);
mov_xy(10,  0) M4(20);
mov_xy(10, 10) M4(16);
mov_xy(10, 20) M4(12);
mov_xy(10, 30) M4( 8);
mov_xy(20,  0) M5(20);
mov_xy(20, 10) M5(16);
mov_xy(20, 20) M5(12);
mov_xy(20, 30) M5( 8);
mov_xy(30,  0) M6(20);
mov_xy(30, 10) M6(16);
mov_xy(30, 20) M6(12);
mov_xy(30, 30) M6( 8);
mov_xy( 0,-10) M3_nut();
mov_xy(10,-10) M4_nut();
mov_xy(20,-10) M5_nut();
mov_xy(30,-10) M6_nut();
mov_xy( 0,-20) M3_washer();
mov_xy(10,-20) M4_washer();
mov_xy(20,-20) M5_washer();
mov_xy(30,-20) M6_washer();