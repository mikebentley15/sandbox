#!/usr/bin/env python3
'''
Explore the clash royalle api
'''

from collections import Counter
from datetime import datetime
import argparse
import copy
import json
import requests
import subprocess as subp
import sys
import tempfile
import urllib
import xml.etree.cElementTree as ET

# TODO: support /players/<tag>/upcomingchests
# TODO: support /players/<tag>/battlelog
# TODO: support /tournaments
# TODO: support /tournaments/<tag>
# TODO: support /cards
# TODO: support /locations
# TODO: support /locations/<tag>
# TODO: support /locations/<tag>/rankings/clans
# TODO: support /locations/<tag>/rankings/players
# TODO: support /locations/<tag>/rankings/clanwars

API = 'https://api.clashroyale.com/v1'
MYCLAN_NAME = 'LDS WarMonGeRs!'
MYCLAN_TAG = '#P98LYC'
MYPLAYER_NAME = 'mikebob'
MYPLAYER_TAG = '#98CVC20C'

class Location:
    def __init__(self, data):
        self.__dict__ = copy.copy(data)
        self.data = data

class Arena:
    def __init__(self, data):
        self.__dict__ = copy.copy(data)
        self.data = data

class Standing:
    def __init__(self, token, data):
        self.__dict__ = copy.copy(data)
        self.data = data
        if 'clan' in data:
            self.clan = Clan(token, self.clan)

class Player:
    def __init__(self, token, data):
        self.__dict__ = copy.copy(data)
        self.data = data
        self.token = token
        if 'arena' in data:
            self.arena = Arena(self.arena)

    @staticmethod
    def get_player(token, tag):
        'Returns a detailed player with statistics'
        tag = urllib.parse.quote_plus(tag)
        code, reply = request_clash(token, '/players/' + tag)
        assert code == 200
        return Player(token, reply)

    def expand(self):
        'Takes a less populated player and expands all fields'
        other = Player.get_player(self.token, self.tag)
        self.data.update(other.data)
        other.data = self.data
        self.__dict__.update(other.__dict__)

class Clan:
    def __init__(self, token, data):
        self.__dict__ = copy.copy(data)
        self.token = token
        self.data = data
        if 'location' in data:
            self.location = Location(self.location)
        if 'memberList' in data:
            self.memberList = [Player(token, x) for x in self.memberList]
    
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

    def expand(self):
        'Takes a minimal clan and expands it (i.e. one from search_clans())'
        other = Clan.get_clan(self.token, self.tag)
        self.data.update(other.data)
        other.data = self.data
        self.__dict__.update(other.__dict__)

    def donation_table(self):
        'Returns the table for donations as (header_row, rows)'
        header_row = ('Name', 'Donations')
        rows = [(x.name, x.donations) for x in self.memberList]
        rows.sort(key=lambda x: x[1])
        rows = [(x[0], str(x[1])) for x in rows]
        return header_row, rows

    def warlog_participation_table(self):
        '''
        Returns the participation table for the warlog as (header_row, rows)
        
        Each column represents a past war.  The possible values are:
        - '': Blank means you did not participate in the battle
        - 'o': You fully participated, fulfilling 3 collection battles
          and 1 war battle
        - 'X': You participated, but missed one of your battles
        - '?': You participated, but it is unknown if you fully participated.
          This happens sometimes when the clash api reports bad numbers
        '''
        warlog = Warlog.get_warlog(self)
        header_row = ['Name'] + ['W' + str(i+1) for i in range(len(warlog.wars))]
        tracker = {x.name : ['']*len(warlog.wars) for x in self.memberList}
        for i, war in enumerate(warlog.wars):
            for member in war.participants:
                if member.name not in tracker:
                    continue
                fully_participated = \
                    member.collectionDayBattlesPlayed == 3 and \
                    member.battlesPlayed > 0
                unknown_participation = \
                    member.collectionDayBattlesPlayed == 0 and \
                    member.battlesPlayed > 0
                if fully_participated:
                    status = 'o'
                elif unknown_participation:
                    status = '?'
                else:
                    status = 'X'
                tracker[member.name][i] = status
        rows = [[key] + val for key, val in tracker.items()]
        rows.sort(key=lambda x: -x.count('o')*len(warlog.wars)*2 - x.count('?')) 
        return header_row, rows

#    def warlog_activity_table(self):
#        '''
#        Returns the activity table for the warlog (i.e. missing battles) as
#        (header_row, rows)
#        '''
#        warlog = Warlog.get_warlog(self)
#        header_row = ['Name', 'Full Collections', 'Full War', 'Missing #']
#        full_collections = Counter()
#        full_war = Counter()
#        missing_any = Counter()
#        for war in warlog.wars:
#            

    def check_donations(self, threshold):
        'Prints the members below the donation threshold'
        below = [x for x in self.memberList if x.donations < threshold]
        below.sort(key=lambda x: x.donations)
        print('Those below the donation threshold of {}:'.format(threshold))
        for member in below:
            print('  {x.name}: {x.donations} donations'.format(x=member))

    def check_current_war(self):
        'Checks to see who has not done all of their battles'
        war = CurrentWar.get_current_war(self)
        print('Current war:')
        print('  state:  ', war.state)
        print('  ends:   ',
              war.collectionEndTime if war.state == 'collectionDay'
              else war.warEndTime)
        below_collection = [x for x in war.participants
                            if x.collectionDayBattlesPlayed < 3
                            and x.collectionDayBattlesPlayed != 0]
        below_collection.sort(key=lambda x: x.collectionDayBattlesPlayed)
        print('  Not finished collection battles:')
        for member in below_collection:
            print('    {x.name}: {x.collectionDayBattlesPlayed}/3'
                  .format(x=member))
        participant_names = set(x.name for x in war.participants)
        not_participating = [x for x in self.memberList
                             if x.name not in participant_names]
        not_participating.sort(key=lambda x: x.name)
        if war.state == 'warDay':
            no_battle_names = [x.name for x in war.participants
                               if x.battlesPlayed == 0]
            print('  Not done with war day battle (cannot see when 2 are available):')
            for name in sorted(no_battle_names):
                print('    {}'.format(name))
        print('  Not participating:')
        for member in not_participating:
            print('    {}'.format(member.name))

    def check_war_log(self):
        'Checks those with bad participation in the war log and prints'
        warlog = Warlog.get_warlog(self)
        print('Checking the warlog back in the past {} wars:'
              .format(len(warlog.wars)))
        bad_collections = Counter()
        bad_battles = Counter()
        bad_any = Counter()
        no_participation = Counter()
        #with open('warlog.json', 'w') as outfile:
        #    json.dump([x.data for x in warlog.wars], outfile, indent=2)
        for war in warlog.wars:
            below_collection_names = [x.name for x in war.participants
                                      if x.collectionDayBattlesPlayed < 3
                                      and x.collectionDayBattlesPlayed != 0]
            # Note: cannot determine when 2 war-day battles are available, but
            #       only one was played.
            below_battles_names = [x.name for x in war.participants
                                   if x.battlesPlayed == 0]
            participant_names = {x.name for x in war.participants}
            not_participating_names = \
                {x.name for x in self.memberList}.difference(participant_names)

            for name in below_collection_names:
                bad_collections[name] += 1
            for name in below_battles_names:
                bad_battles[name] += 1
            for name in set(below_collection_names).union(below_battles_names):
                bad_any[name] += 1
            for name in not_participating_names:
                no_participation[name] += 1

        def print_counter(counter, headers=None):
            key_len = max(len(str(x)) for x in counter)
            val_len = 1
            format_str = '    {:<{keywidth}s}   {:^{valwidth}}'
            if headers is not None:
                key_len = max(key_len, len(headers[0]))
                val_len = max(1, len(headers[1]))
                print(format_str.format(headers[0], headers[1],
                                        keywidth=key_len, valwidth=val_len))
                print(format_str.format('-'*key_len, '-'*val_len,
                                        keywidth=key_len, valwidth=val_len))
            for name, val in counter.most_common():
                print(format_str.format(name, val,
                                        keywidth=key_len, valwidth=val_len))

        print('  Those missing collection battles:')
        print_counter(bad_collections, ('NAME', '# MISSING'))
        print('  Those missing war day battles: (cannot determine 1/2 case)')
        print_counter(bad_battles, ('NAME', '# MISSING'))
        print('  Those missing any battle (i.e. combining the other categories):')
        print_counter(bad_any, ('NAME', '# MISSING'))
        print('  Those who did not participate in some wars:')
        print_counter(no_participation, ('NAME', '# MISSING'))

    def __repr__(self):
        return repr(self.data)

class WarlogEntry:
    def __init__(self, token, data):
        self.__dict__ = copy.copy(data)
        self.data = data
        self.token = token
        if 'participants' in data:
            self.participants = [Player(token, x) for x in self.participants]
        if 'standings' in data:
            self.standings = [Standing(token, x) for x in self.standings]
        if 'createdDate' in data:
            self.createdDate = parse_datetime_str(self.createdDate)

class Warlog:
    _CACHED_WARLOGS = {}

    def __init__(self, entries):
        self.wars = entries

    @staticmethod
    def get_warlog(clan):
        "Returns the clan's warlog"
        if clan.tag in Warlog._CACHED_WARLOGS:
            return CACHED_WARLOGS[clan.tag]
        tag = urllib.parse.quote_plus(clan.tag)
        code, reply = request_clash(clan.token, '/clans/' + tag + '/warlog')
        assert code == 200
        warlog = Warlog([WarlogEntry(clan.token, x) for x in reply['items']])
        Warlog._CACHED_WARLOGS[clan.tag] = warlog
        return warlog

class CurrentWar:
    def __init__(self, token, data):
        self.__dict__ = copy.copy(data)
        self.data = data
        self.token = token
        if 'collectionEndTime' in data:
            self.collectionEndTime = parse_datetime_str(self.collectionEndTime)
        if 'warEndTime' in data:
            self.warEndTime = parse_datetime_str(self.warEndTime)
        if 'clan' in data:
            self.clan = Clan(token, self.clan)
        if 'participants' in data:
            self.participants = [Player(token, x) for x in self.participants]

    @staticmethod
    def get_current_war(clan):
        'Returns the current war statistics for the clan'
        tag = urllib.parse.quote_plus(clan.tag)
        code, reply = request_clash(clan.token, '/clans/' + tag + '/currentwar')
        assert code == 200
        return CurrentWar(clan.token, reply)

def parse_args(arguments):
    'Parse arguments with argparse'
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--token', metavar='tokenfile', required=True,
                        dest='tokenfile', help='File containing token')
    parser.add_argument('-o', '--output', help='Output file')
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

def parse_datetime_str(clash_datetime):
    '''
    Converts a datetime string from the Clash API into a datetime object.
    The string is in the form:
      YYYYmmddTHHMMSS.sssZ
    '''
    return datetime.strptime(clash_datetime[:-1] + '000', '%Y%m%dT%H%M%S.%f')

def create_html_table(rows, header=None, htmlclass=None, htmlid=None):
    '''
    Output an HTML table from the rows.

    The table will have classes for odd and even rows called "odd" and "even"
    to be styled with CSS.

    @param rows: (list(list(str))) The table rows.  Each internal list
        must be of the same length.
    @param header: (list(str)) The header row.  Must be of the same length as
        each row in rows.  If None, then there will be no header row.
    @param htmlclass: (str) The value for the class attribute in the html
        table.
    @param htmlid: (str) The value for the id attribute in the html table.

    >>> create_html_table([['a', 'b', 'c']])
    '<table><tbody><tr class="odd"><td>a</td><td>b</td><td>c</td></tr></tbody></table>'
    '''
    assert all(len(rows[0]) == len(row) for row in rows)
    assert header is None or len(header) == len(rows[0])

    root = ET.Element('table')
    if htmlclass is not None:
        root.set('class', htmlclass)
    if htmlid is not None:
        root.set('id', htmlid)

    if header is not None:
        head = ET.SubElement(root, 'thead')
        tr = ET.SubElement(head, 'tr')
        tr.set('class', 'head')
        for head_item in header:
            th = ET.SubElement(tr, 'th')
            th.text = head_item

    body = ET.SubElement(root, 'tbody')
    for i, row in enumerate(rows):
        i += 1
        tr = ET.SubElement(body, 'tr')
        if i % 2 == 0:
            tr.set('class', 'even')
        else:
            tr.set('class', 'odd')
        for elem in row:
            td = ET.SubElement(tr, 'td')
            td.text = elem

    return ET.tostring(root, encoding='unicode')

def text_output(token, outfilepath=None):
    'Output in text format'
    myclan = Clan.get_clan(token, MYCLAN_TAG)
    from pprint import pprint
    #pprint(myclan.data)
    try:
        origstdout = sys.stdout
        if outfilepath is not None:
            sys.stdout = open(outfilepath, 'w')
        myclan.check_donations(100)
        myclan.check_current_war()
        myclan.check_war_log()
    finally:
        if outfilepath is not None:
            sys.stdout.close()
        sys.stdout = origstdout

def html_output(token, outfilepath=None):
    'Output in HTML format'
    myclan = Clan.get_clan(token, MYCLAN_TAG)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.html') as html_orig:
        print('<html><head></head><body>', file=html_orig)
        hdr, rows = myclan.donation_table()
        print(create_html_table(rows, header=hdr), file=html_orig)
        hdr, rows = myclan.warlog_participation_table()
        print(create_html_table(rows, header=hdr), file=html_orig)
        print('</body></html>', file=html_orig)
        html_orig.flush()
        output = subp.check_output(['xmllint', '--format', html_orig.name])
        output = output.decode('utf-8')
    if outfilepath is not None:
        with open(outfilepath, 'w') as outfile:
            print(output, file=outfile)
    else:
        print(output)

def main(arguments):
    'Main logic here.  Call with --help for info'
    args = parse_args(arguments)
    #text_output(args.token, args.output)
    html_output(args.token, args.output)

if __name__ == '__main__':
    main(sys.argv[1:])
