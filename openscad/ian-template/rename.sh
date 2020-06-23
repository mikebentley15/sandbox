#!/bin/bash

NAME=$1

echo $1

sed -i "s|Template|$1|g" Makefile
sed -i "s|Template|$1|g" Template.scad
mv Template.scad ${1}.scad

rm rename.sh
