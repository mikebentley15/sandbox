#!/usr/bin/env python
'Cleans duplicates out of the pacman directory tree'

import os
import re
import shutil
import sys

def main(arguments):
    'Main logic here'
    pacman_dir = '/var/lib/pacman/local'
    packages = os.listdir(pacman_dir)
    packages.sort

    old_dir = '/var/lib/pacman/OLD'
    os.makedirs(old_dir)

    pkgname_search = re.compile('^(.*?)-[0-9]')
    
    old_packages = []
    for pkg1 in packages:
        if pkg1 in old_packages:
            continue
        pkgname = pkgname_search.finall(pkg)[0]
        for pkg2 in packages:
            if pkg2 == pkg1:
                continue
            if pkg2 in old_packages:
                continue
            if pkgname == pkgname_search.finall(pkg2)[0]:
                # we have found duplicate packages
                path1 = os.path.join(pacman_dir, pkg1)
                path2 = os.path.join(pacman_dir, pkg2)
                old_package = pkg1
                if os.stat(path1).st_mtime > os.stat(path2).st_mtime:
                    old_package = pkg2
                old_packages.append(old_package)
                oldpath = os.path.join(pacman_dir, old_package)
                target = os.path.join(old_dir, old_package)
                if os.path.exists(oldpath):
                    print('move {} -> {}'.format(oldpath, target))
                    shutil.move(oldpath, target)

if __name__ == '__main__':
    main(sys.argv[1:])
