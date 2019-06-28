#!/usr/bin/env python3
'''
This is a minimization delta debugging algorithm with an example to find the
minimal set to cause inkscape to fail on this buggy svg.

Bug description:
    given this buggy svg file, open the file, and delete the 'Notes-M' layer as
    the first action.  This causes Inkscape to crash with the following error
    to the console:
        Emergency save activated!
        Emergency save completed. Inkscape will close now.
        If you can reproduce this crash, please file a bug at www.inkscape.org
        with a detailed description of the steps leading to the crash, so we can fix it.
        Segmentation fault (core dumped)

The goal of using this delta debugging algorithm is to find the minimal svg
that still causes this failure.  There will be three outcomes to the test:

1. The program still crashes (failure)
2. The program succeeds (pass)
3. The xml is not valid or the 'Notes-M' layer does not exist (undetermined)

The goal is the make the minimal set that causes the crash.  I can easily check
to see if the xml is valid using xmllint.  I can do a rough guess at how to
check for the 'Notes-M' layer existing.  The real challenging part is to
automate launching the inkscape GUI and deleting that layer.
'''

import xml.etree.ElementTree as ET
import sys
import re
import subprocess as subp
import os
import time

def test_inkscape(content):
    '''
    Tests if the problem is still there (see file docstring).  Returns one of
    three values

    1. True: this means the test passed (meaning the crash did not occur)
    2. False: this means the crash still occurred
    3. None: the problem was ill-formed.  For this case, it means either the
       xml is not valid, or the 'Notes-M' layer does not exist.
    '''
    if isinstance(content, list):
        content = '\n'.join(content)
    try:
        root = ET.fromstring(content)
        #print_heirarchy(root)
    except ET.ParseError:
        return None

    if not layer_exists(root, 'Notes-M'):
        return None

    with open('tmp.svg', 'w') as fout:
        fout.write(content)

    # start the inkscape process
    p = subp.Popen(['inkscape', 'tmp.svg'], stdout=subp.DEVNULL, stderr=subp.DEVNULL)

    # send the required X11 events
    import Xlib
    import Xlib.display
    # wait for a while
    time.sleep(2)
    # left click at (1629, 208)
    #subp.run(['xte'], input=b'mousemove 1629 208\nmouseclick 1\n')
    subp.run(['xte'], input=b'mousemove 401 191\nmouseclick 1\n')
    time.sleep(0.3)
    # left click at (1125, 570)
    subp.run(['xte'], input=b'mousemove 1125 570\nmouseclick 1\n')
    time.sleep(1)

    p.terminate()
    p.communicate()

    print('Inkscape Return Code:', p.returncode, ', content size:', len(content))
    return p.returncode != -11
    

def layer_exists(element, layer_name):
    '''
    Returns True if there is an inkscape layer with the layer_name at or below
    the given XML element.
    '''
    ns = {
        'inkscape': 'http://www.inkscape.org/namespaces/inkscape',
        'svg': 'http://www.w3.org/2000/svg',
        }
    label = '{' + ns['inkscape'] + '}label'
    return layer_name in \
            [x.attrib[label] for x in element.findall('.//svg:g[@inkscape:label]', ns)]

def print_heirarchy(element, indent=' ', depth=0):
    tag = re.sub('\{.*\}', '', element.tag)
    tag = element.tag
    attrib = None
    name = '{http://www.inkscape.org/namespaces/inkscape}label'
    if tag == '{http://www.w3.org/2000/svg}g' and name in element.attrib:
        attrib = element.attrib[name]
    if tag == 'path':
        return
    if attrib is not None:
        print('{}<{}>: {}'.format(indent*depth, tag, attrib))
    else:
        print('{}<{}>'.format(indent*depth, tag))
    for child in element:
        print_heirarchy(child, indent, depth+1)
    #print('{}</{}>'.format(indent*depth, tag))

def print_click_events():
    import Xlib
    import Xlib.display
    display = Xlib.display.Display(':0')
    root = display.screen().root
    root.change_attributes(event_mask=
        Xlib.X.ButtonPressMask | Xlib.X.ButtonReleaseMask)

    while True:
        event = root.display.next_event()
        print(event)

def dd(content, test):
    '''
    The delta-debugging algorithm
    '''
    memoizer = {}
    def test_wrapped(inside_content):
        if isinstance(inside_content, str):
            as_string = inside_content
        elif isinstance(inside_content, list):
            if inside_content and isinstance(inside_content[0], ET.Element):
                as_string = '\n'.join(ET.tostring(x).decode('utf-8') for x in inside_content)
            else:
                as_string = '\n'.join(str(x) for x in inside_content)
        else:
            as_string = str(inside_content)
        index = (len(as_string), hash(as_string))
        if index in memoizer:
            return memoizer[index]
        result = test(inside_content)
        memoizer[index] = result
        return result

    return _dd_rec(content, test_wrapped)

def _dd_rec(content, test, n=2):
    '''
    The delta-debugging algorithm recursive implementation
    '''
    assert test(content) is False

    if len(content) == 1:
        return content

    delta_size = len(content) // n

    # try a subset
    for i in range(n):
        delta = content[delta_size*i:delta_size*(i+1)]
        if test(delta) is False:
            return _dd_rec(delta, test, 2)

    # try a complement of a subset
    for i in range(n):
        delta = content[:delta_size*i] + content[delta_size*(i+1):]
        if test(delta) is False:
            return _dd_rec(delta, test, max(2, n-1))

    # try splitting up into smaller pieces
    if n < len(content):
        return _dd_rec(content, test, min(len(content), 2*n))

    # otherwise, done
    return content

def main(arguments):
    #print_click_events()
    content = open(arguments[0], 'r').read()
    #content = open(arguments[0], 'r').readlines()
    #print('Test Result:', test(content))
    min_content = dd(content, test_inkscape)
    print()
    print('minimal svg file:')
    print(min_content)

if __name__ == '__main__':
    main(sys.argv[1:])
