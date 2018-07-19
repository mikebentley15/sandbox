#!/usr/bin/env python3
'''
This is a minimization heirarchical delta debugging algorithm with an example
to find the minimal set to cause inkscape to fail on this buggy svg.

Bug description:
    given this buggy svg file, open the file, and delete the 'Notes-M' layer as
    the first action.  This causes Inkscape to crash with the following error
    to the console:
        Emergency save activated!
        Emergency save completed. Inkscape will close now.
        If you can reproduce this crash, please file a bug at www.inkscape.org
        with a detailed description of the steps leading to the crash, so we can fix it.
        Segmentation fault (core dumped)

The goal of using this heirarchical delta debugging algorithm is to find the
minimal svg that still causes this failure.  There will be two outcomes to
the test:

1. The program still crashes (failure)
2. The program succeeds (pass)

The third outcome of an invalid xml file will no longer be the case here since
the heirarchical delta debugging algorithm only searches over valid inputs.

The goal is the make the minimal set that causes the crash.  I don't know if I
want to check that the 'Notes-M' layer exists.  The real challenging part is to
automate launching the inkscape GUI and deleting that layer.  But that has
already been done in dd.py, so I will leverage that solution there.
'''

import xml.etree.ElementTree as ET
import sys
import re
import subprocess as subp
import os
import time
from tempfile import NamedTemporaryFile
import copy

import dd

def xml_prune(root, level, to_keep):
    '''
    Prunes from an ElementTree.Element object (root) at the given level, to
    throw away everything not found in the to_keep list (or tuple).  Returns
    the pruned root as a copy.

    >>> root = ET.fromstring('<doc><a><b/></a><a/></doc>')
    >>> root_0 = prune(root, 1, [root[0]])
    >>> root_1 = prune(root, 1, [root[1]])
    >>> ET.tostring(root)
    b'<doc><a><b /></a><a /></doc>'
    >>> ET.tostring(root_0)
    b'<doc><a><b /></a></doc>'
    >>> ET.tostring(root_1)
    b'<doc><a /></doc>'
    '''
    if level < 1:
        raise IndexError('Cannot prune higher than level 1')
    root_copy = copy.deepcopy(root)
    nodes = xml_extract_level(root, level - 1)
    nodes_copy = xml_extract_level(root_copy, level - 1)
    for i, node in enumerate(nodes):
        to_delete = []
        for j, child in enumerate(node):
            if child not in to_keep:
                to_delete.append(j)
        for j in reversed(to_delete):
            del nodes_copy[i][j]
    return root_copy

def xml_extract_level(root, level):
    '''
    Returns a list of nodes at the level of the tree from the root.

    @param root: an ElementTree.Element object
    @param level: int, specifying the level to pull.  Level 0 is the root node.

    >>> root = ET.fromstring('<doc><a><b/></a><a/></doc>')
    >>> L = extract_level(root, 1)
    >>> [ET.tostring(x) for x in L]
    [b'<a><b /></a>', b'<a />']
    '''
    nodes = [root]
    for _ in range(level):
        old_nodes = nodes
        nodes = []
        for node in old_nodes:
            nodes.extend(node[:])
    return nodes

def test_inkscape(content):
    '''
    Tests if the problem is still there (see file docstring).  Returns one of
    three values

    1. True: this means the test passed (meaning the crash did not occur)
    2. False: this means the crash still occurred
    3. None: the problem was ill-formed.  For this case, it means either the
       xml is not valid, or the 'Notes-M' layer does not exist.
    '''
    if not dd.layer_exists(content, 'Notes-M'):
        return None

    with NamedTemporaryFile(delete=False, dir='.', suffix='.svg') as fout:
        fout.write(ET.tostring(content))
        fname = fout.name

    # start the inkscape process
    p = subp.Popen(['inkscape', fname], stdout=subp.DEVNULL, stderr=subp.DEVNULL)

    # send the required X11 events
    #import Xlib
    #import Xlib.display
    # wait for a while
    #time.sleep(2)
    # left click at (1629, 208)
    #subp.run(['xte'], input=b'mousemove 1629 208\nmouseclick 1\n')
    #subp.run(['xte'], input=b'mousemove 401 191\nmouseclick 1\n')
    #time.sleep(0.3)
    # left click at (1125, 570)
    #subp.run(['xte'], input=b'mousemove 1125 570\nmouseclick 1\n')
    #time.sleep(1)

    # Manually run the test

    #p.terminate()
    p.communicate()

    print('    Inkscape Return Code:', p.returncode,
          ', content size:', len(content))
    return p.returncode != -11

def hdd(root, test, prune, extract_level):
    '''
    Performs the heirarchical delta debugging algorithm.  This actually will
    make use of the original delta debugging algorithm as it is without
    alterations, but wraps around it to be knowledgeable about the heirarchical
    structure.

    @param root: the root of the tree heirarchy
    @param test: function that takes the root element and return False if the
        test fails.
    @param prune: function that takes the root, the level, and a list of nodes
        to keep from that level, and returns a copy of root with everything
        else from that level pruned away.
    @param extract_level: function that takes the root and the level and
        returns a list of nodes from that level in the heirarchy (references).

    @return a copy of root with the pruned out nodes to be a minimal tree that
        still causes the test to fail (i.e. test_gen(root, 0)([root]) returns
        false)
    '''
    print('Heirarchical Delta Debugging')
    assert test(root) is False
    level = 1
    nodes = extract_level(root, level)
    while nodes:
        print('  Level:', level)
        level_test = lambda nodes: test(prune(root, level, nodes))
        to_keep = dd.dd(nodes, level_test)
        root = prune(root, level, to_keep)
        level += 1
        nodes = extract_level(root, level)
    return root

def main(arguments):
    #print_click_events()
    #content = open(arguments[0], 'r').read()
    #content = open(arguments[0], 'r').readlines()
    content = ET.parse(arguments[0]).getroot()
    #print('Test Result:', test_inkscape(content))
    min_content = hdd(content, test_inkscape, xml_prune, xml_extract_level)
    #print()
    print('minimal svg file:')
    print(ET.tostring(min_content))
    with open('min.svg', 'w') as fout:
        fout.write(ET.tostring(min_content).decode('utf-8'))

if __name__ == '__main__':
    main(sys.argv[1:])
