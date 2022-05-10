#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import sys
import argparse
import contextlib

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Remove clipPath tags and clip-path attributes from all elements.
        '''
    parser.add_argument('input', nargs='?', default=None,
            help='Input file (or standard in if not specified)')
    parser.add_argument('output', nargs='?', default=None,
            help='Output file (or standard out if not specified)')
    parser.add_argument('-i', '--in-place', action='store_true',
            help='''
                If not given an output file, this option says to replace the
                contents of the input file.  You cannot give an output file and
                this option.  Note: this option is equivalent to giving the
                same file as both the input and output arguments.
            ''')
    return parser

class NsTreeBuilder(ET.TreeBuilder):
    '''
    Same as ET.TreeBuilder except for the following:
    - default for insert_comments and insert_pis is changed to True
    - nsmap: added property, dict of namespace to url from the parse
    '''
    def __init__(self, element_factory=None, *,
                 insert_comments=True,
                 insert_pis=True,
                 **kwargs):
        super().__init__(
            element_factory=element_factory,
            insert_comments=insert_comments,
            insert_pis=insert_pis,
            **kwargs,
        )
        self.nsmap = {}

    def start_ns(self, prefix, uri):
        assert prefix not in self.nsmap
        self.nsmap[prefix] = uri

def parse_xml_with_ns(source):
    '''
    Parses the XML file, returning an ElementTree and a namespace dict (prefix
    -> url).
    '''
    builder = NsTreeBuilder()
    parser = ET.XMLParser(target=builder)
    tree = ET.ElementTree()
    tree.parse(source, parser=parser)
    namespaces = builder.nsmap
    return tree, namespaces

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    fin = sys.stdin if args.input is None else args.input
    if args.in_place:
        assert args.output is None
        assert args.input is not None
    if args.output is not None:
        fout = args.output
    elif args.in_place and args.input is not None:
        fout = args.input
    else:
        fout = sys.stdout

    tree, namespaces = parse_xml_with_ns(fin)

    for namespace, url in namespaces.items():
        ET.register_namespace(namespace, url)

    root = tree.getroot()

    namespaces['svg'] = 'http://www.w3.org/2000/svg'

    # remove clippath elements
    for parent in tree.findall('.//svg:clipPath/..', namespaces):
        for child in parent.findall('./svg:clipPath', namespaces):
            parent.remove(child)

    # remove clip-path attributes
    for element in root.findall('.//*[@clip-path]'):
        del(element.attrib['clip-path'])

    tree.write(fout, encoding='unicode')

if __name__ == '__main__':
    main(sys.argv[1:])
