#!/usr/bin/env python3
'''
Explore the clash royalle api
'''

import argparse
import sys
import requests
import json
import urllib
import copy

HEADER = 'authorization: Bearer {}'
API = 'https://api.clashroyale.com/v1'
MYCLAN_NAME = 'LDS WarMonGeRs!'
MYCLAN_TAG = '#P98LYC'

class Location:
    def __init__(self, data):
        self.__dict__ = copy.copy(data)
        self.data = data

class Arena:
    def __init__(self, data):
        self.__dict__ = copy.copy(data)
        self.data = data

class Member:
    def __init__(self, data):
        self.__dict__ = copy.copy(data)
        self.data = data
        if 'arena' in data:
            self.arena = Arena(self.arena)

class Clan:
    def __init__(self, token, data):
        self.__dict__ = copy.copy(data)
        self.token = token
        self.data = data
        if 'location' in data:
            self.location = Location(self.location)
        if 'memberList' in data:
            self.memberList = [Member(x) for x in self.memberList]
    
    @staticmethod
    def search_clans(token, **kwargs):
        '''
        Returns a list of clans from the search criteria.  Not all clan details
        are here.  Things you can search in the kwargs are:
        - name
        - locationId
        - minMembers
        - maxMembers
        - minScore
        - limit
        - after
        - before
        '''
        code, reply = request_clash(token, '/clans', params=kwargs)
        assert code == 200
        items = reply.json()['items']
        return [Clan(token, x) for x in items]

    @staticmethod
    def get_clan(token, tag):
        'Returns a detailed clan with members and such'
        tag = urllib.parse.quote_plus(tag)
        code, reply = request_clash(token, '/clans/' + tag)
        assert code == 200
        return Clan(token, reply)

    def expand_clan(self):
        'Takes a minimal clan and expands it (i.e. one from search_clans())'
        other = Clan.get_clan(self.token, self.tag)
        self.__dict__ = other.__dict__

    def __repr__(self):
        return repr(self.data)


def parse_args(arguments):
    'Parse arguments with argparse'
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--token', metavar='tokenfile', dest='tokenfile',
                        help='File containing token')
    args = parser.parse_args(arguments)
    with open(args.tokenfile, 'r') as fin:
        args.token = fin.read().strip()
    return args

def request_clash(token, path, params=None, headers=None):
    '''
    Performs the request.
    
    @param token: (str) the token used to authenticate as you
    @param path: (str) the path to the desired api (e.g. '/clans')
    @param params: (dict) the parameters to send with get
    @param headers: (dict) headers
    
    @return (status_code, json_obj)
    '''
    if headers is None:
        headers = dict()
    headers['authorization'] = 'Bearer {}'.format(token)
    reply = requests.get(API + path, params=params, headers=headers)
    return (reply.status_code, reply.json())

def main(arguments):
    'Main logic here.  Call with --help for info'
    args = parse_args(arguments)
    myclan = Clan.get_clan(args.token, MYCLAN_TAG)
    from pprint import pprint
    pprint(myclan.data)

if __name__ == '__main__':
    main(sys.argv[1:])
