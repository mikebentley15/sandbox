This directory is my starting template for OpenSCAD projects.

The intended use is to copy the entire directory, then run `rename.sh <new_name>`
in the new directory.

The script renames the files and variables then removes itself.

Once that is done three manual changes are required.

1. In the scad file change the options in the comment after 'part' to be what
   parts will be in your design.

2. Change the code after the handy functions section to conditionally generate
   the parts defined in 1. I recomend using the same pattern as the default
   with a conditional before the module which generates the part.

3. In the makefile change the 'MODELS' line to be the part names defined in 1.
   with '.stl' on the end.


Usefull links:

- https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/The_OpenSCAD_Language
- https://www.openscad.org/cheatsheet/
- https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Customizer


Notes:

* If two parts are to fit together for 3d printing I reccomend a ~0.2 mm gap
  between them.

* Smoothness of curved surfaces are affeccted by multiple special variables, I
  prefer modifying $fn, which is the number of segments.

* Scope is a particularly annoying point with OpenSCAD, you can assign to a
  variable multiple times, but only the last will actually be used.

eg:
  x = 0;
  echo(x);
  x = 1;
  echo(x);

will print:
 ECHO: 1
 ECHO: 1
