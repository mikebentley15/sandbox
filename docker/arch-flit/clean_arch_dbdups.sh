#!/bin/bash

set -e
set -u

pacman_dir=/var/lib/pacman/local
old_dir=/var/lib/pacman/OLD

mkdir -p ${old_dir}

# Take package dir name and strip off version number
function pkgname() {
  echo "$1" | sed -e 's/-.*//g'
}

cd ${pacman_dir}

for pkgdir in *; do
  if [ ! -d "${pkgdir}" ]; then
    continue
  fi
  name=$(pkgname "${pkgdir}")
  
  for pkg2 in ${name}-*; do
    name2=$(pkgname "${pkg2}")
    if [ "${pkgdir}" == "${pkg2}" ]; then
      continue;
    elif [ ! -d "${pkg2}" ]; then
      continue;
    elif [ "${name}" == "${name2}" ]; then
      # found duplicate
      old_package=$(echo -e "${pkgdir}\n${pkg2}" | sort --version-sort | head -n 1)
      if [ -d "${old_package}" ]; then
        echo "${old_package} -> ${old_dir}/"
        mv "${old_package}" "${old_dir}"
      fi
    fi
  done
done
