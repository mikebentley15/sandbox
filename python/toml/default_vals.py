import toml
import pprint


config_provided = '''
# This is a TOML document.

value = 'TOML Example'
'''

config_not_provided = '''
# This is a TOML document.

#value = 'TOML Example'
'''

pp = pprint.PrettyPrinter(width=80, indent=2)
def ptoml(name, *args, **kwargs):
    print(name)
    try:
        config = toml.loads(*args, **kwargs)
    except toml.TomlDecodeError:
        print('  Parsing error')
    else:
        pp.pprint(config)
    print()

#ptoml('Provided value',    config_provided)
#ptoml('No provided value', config_not_provided)

class DefaultDict(dict):
    def __init__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)
        self['value'] = 'TOML Example'

#ptoml('Provided value with DefaultDict', config_provided, _dict=DefaultDict)
#ptoml('No provided value with DefaultDict', config_not_provided, _dict=DefaultDict)

def loads_toml(string, defaultstring):
    config = toml.loads(string)
    defaults = toml.loads(defaultstring)
    pass # ...

defaultstring = '''
# This is a TOML document.

title = "TOML Example"

[owner]
name = "Tom Preston-Werner"
dob = 1979-05-27T07:32:00-08:00 # First class dates

[database]
server = "192.168.1.1"
ports = [ 8001, 8001, 8002 ]
connection_max = 5000
enabled = true

[servers]

  # Indentation (tabs and/or spaces) is allowed but not required
  [servers.alpha]
  ip = "10.0.0.1"
  dc = "eqdc10"

  [servers.beta]
  ip = "10.0.0.2"
  dc = "eqdc10"

[clients]
data = [ ["gamma", "delta"], [1, 2] ]

# Line breaks are OK when inside arrays
hosts = [
  "alpha",
  "omega"
]
'''

emptystring = ''

#ptoml('Default string', defaultstring)
#ptoml('Empty string', emptystring)

defaultstring = '''
# -- LICENSE BEGIN --
#
# Copyright (c) 2015-2018, Lawrence Livermore National Security, LLC.
#
# Produced at the Lawrence Livermore National Laboratory
#
# Written by
#   Michael Bentley (mikebentley15@gmail.com),
#   Geof Sawaya (fredricflinstone@gmail.com),
#   and Ian Briggs (ian.briggs@utah.edu)
# under the direction of
#   Ganesh Gopalakrishnan
#   and Dong H. Ahn.
#
# LLNL-CODE-743137
#
# All rights reserved.
#
# This file is part of FLiT. For details, see
#   https://pruners.github.io/flit
# Please also read
#   https://github.com/PRUNERS/FLiT/blob/master/LICENSE
#
# Redistribution and use in source and binary forms, with or
# without modification, are permitted provided that the following
# conditions are met:
#
# - Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the disclaimer below.
#
# - Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the disclaimer
#   (as noted below) in the documentation and/or other materials
#   provided with the distribution.
#
# - Neither the name of the LLNS/LLNL nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL LAWRENCE LIVERMORE NATIONAL
# SECURITY, LLC, THE U.S. DEPARTMENT OF ENERGY OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# Additional BSD Notice
#
# 1. This notice is required to be provided under our contract
#    with the U.S. Department of Energy (DOE). This work was
#    produced at Lawrence Livermore National Laboratory under
#    Contract No. DE-AC52-07NA27344 with the DOE.
#
# 2. Neither the United States Government nor Lawrence Livermore
#    National Security, LLC nor any of their employees, makes any
#    warranty, express or implied, or assumes any liability or
#    responsibility for the accuracy, completeness, or usefulness of
#    any information, apparatus, product, or process disclosed, or
#    represents that its use would not infringe privately-owned
#    rights.
#
# 3. Also, reference herein to any specific commercial products,
#    process, or services by trade name, trademark, manufacturer or
#    otherwise does not necessarily constitute or imply its
#    endorsement, recommendation, or favoring by the United States
#    Government or Lawrence Livermore National Security, LLC. The
#    views and opinions of authors expressed herein do not
#    necessarily state or reflect those of the United States
#    Government or Lawrence Livermore National Security, LLC, and
#    shall not be used for advertising or product endorsement
#    purposes.
#
# -- LICENSE END --
# Autogenerated by "flit init"
#   flit version {flit_version}

[database]

# older versions of flit supported postgres.  that has been removed.  only
# sqlite is supported at the moment.
type = 'sqlite'

# if relative path, it is relative to the directory containing this
# configuration file.
filepath = 'results.sqlite'


[run]

# Set this to false to not do any timing at all
timing = true

# The number of loops to run with the timing.  For values < 0, the number of
# loops to run will be determined automatically.
timing_loops = -1

# How many times to repeat the timing.  The minimum of the repeated timings
# will be kept.
timing_repeats = 3


# For now, only the first host is supported, all others are ignored
[[hosts]]

name = '{hostname}'
flit_path = '{flit_path}'
config_dir = '{config_dir}'

# The settings for "make dev"
[hosts.dev_build]
# compiler_name must be found in [[hosts.compilers]] list under name attribute
# but the optimization level and switches do not need to be in the compiler list
compiler_name = 'g++'
optimization_level = '-O2'
switches = '-funsafe-math-optimizations'

# The ground truth compilation to use in analysis, for "make gt"
[hosts.ground_truth]
# compiler_name must be found in [[hosts.compilers]] list under name attribute
# but the optimization level and switches do not need to be in the compiler list
compiler_name = 'g++'
optimization_level = '-O0'
switches = ''

  # This host's list of compilers.
  # For now, only used for hosts.ground_truth and hosts.dev_build.
  # TODO: use this list to generate the Makefile
  [[hosts.compilers]]

  # binary can be an absolute path, relative path, or binary name (found in
  # PATH).  If you want to specify a compiler in the same directory as this
  # config file, prepend with a "./" (e.g. "./my-compiler")
  binary = 'g++'
  name = 'g++'

'''

config = toml.loads(defaultstring)
empty = {}

import copy
def fill_defaults(vals, defaults):
    '''
    Given two combinations of dictionaries and lists (such as something
    generated from a json file or a toml file), enforce the defaults where the
    vals has missing values.
    
    - For dictionaries, missing keys will be populated with default values
    - For lists, this will recursively fill the defaults on each list item with
      the first list item in defaults (all other list items in defaults are
      ignored)

    Modifies vals and also returns the vals dictionary.

    >>> fill_defaults({'a': 1}, {})
    {'a': 1}

    >>> fill_defaults({}, {'a': 1})
    {'a': 1}

    >>> fill_defaults({'a': 1}, {'a': 2})
    {'a': 1}

    >>> fill_defaults({'a': 2}, {'a': 1, 'b': 3})
    {'a': 2, 'b': 3}

    >>> fill_defaults([{}, {'a': 1}], [{'a': 2, 'b': 3}])
    [{'a': 2, 'b': 3}, {'a': 1, 'b': 3}]
    '''
    if isinstance(vals, dict):
        assert isinstance(defaults, dict)
        for key in defaults:
            if key not in vals:
                vals[key] = copy.deepcopy(defaults[key])
            else:
                fill_defaults(vals[key], defaults[key])
    elif isinstance(vals, list):
        assert isinstance(defaults, list)
        for x in vals:
            fill_defaults(x, defaults[0])
    return vals

ptoml('Default string', defaultstring)
ptoml('Empty string', emptystring)

ptoml('Filled defaults with empty', toml.dumps(fill_defaults(empty, config)))

ptoml('Filled defaults with some run info and host values',
      toml.dumps(
          fill_defaults(toml.loads('''
              [run]
              timing = false
              [[hosts]]
              [hosts.dev_build]
              compiler_name = "clang++"
              [[hosts]]
              [[hosts]]
              compilers = []
              '''), config)))

