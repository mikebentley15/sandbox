#!/usr/bin/env python3

import sys
import xml.etree.ElementTree as ET

def main(files):
    full = ET.Element('dataset')
    for fname in files:
        tree = ET.parse(fname)
        root = tree.getroot()
        for pair in root:
            full.append(pair)

if __name__ == '__main__':
    main(sys.argv[1:])
