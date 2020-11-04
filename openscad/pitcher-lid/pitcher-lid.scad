/* [File] */

file = "lid.svg";

view_color = [0.0, 1.0, 0.7, 0.5];

$fn = 200;

//color(view_color) rotate_extrude($fn=100) import(file);
color(view_color) rotate_extrude() import(file);
