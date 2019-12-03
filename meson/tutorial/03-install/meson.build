project('install', 'c')
mylib = shared_library('mylib', 'libfile.c', install : true)
mylib_dep = declare_dependency(link_with: mylib)
executable('prog', 'prog.c',
           install : true,
           install_dir : 'my/special/dir',
           dependencies : mylib_dep)
install_subdir('mydir', install_dir : 'include')
