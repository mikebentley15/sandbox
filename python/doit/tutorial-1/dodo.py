'Task file for doit command-line tool'

import pathlib

import doit
import pygraphviz as gviz
from import_deps import PyModule, ModuleSet

base_path = pathlib.Path(doit.__file__).parent
PKG_MODULES = ModuleSet(base_path.glob('**/*.py'))

DOIT_CONFIG = {
    # default values are first of the available options
    'default_tasks': ['imports', 'dot', 'draw'],
    'verbosity': 2, # in (1, 0, 2): 1 shows stderr, 2 adds stdout
    'backend': 'sqlite3', # one of ('dbm', 'json', 'sqlite3')
    'dep_file': '.doit.sqlite', # not required
    'check_file_uptodate': 'md5', # one of ('md5', 'timestamp')
    'action_string_formatting': 'both', # one of ('old', 'new', 'both')
}

def task_imports():
    module_path = doit.__file__
    deps_file = 'doit.api.deps'
    v1 = {
        'file_dep': [module_path],
        'targets': [deps_file],
        'actions': [f'python -m import_deps {module_path} > {deps_file}'],
    }
    v2 = {
        'file_dep': [module_path],
        'targets': [deps_file],
        'actions': ['python -m import_deps %(dependencies)s > %(targets)s'],
        'clean': True,
    }
    v3 = {
        'file_dep': [module_path],
        'actions': [(get_imports_v1, [module_path])],
    }
    # v4
    base_path = pathlib.Path(doit.__file__).parent
    pkg_modules = ModuleSet(base_path.glob('**/*.py'))
    for name, module in pkg_modules.by_name.items():
        yield {
            'name': name, # subtask name
            'file_dep': [module.path],
            'actions': [(get_imports_v2, (pkg_modules, module.path))],
        }

def task_dot():
    'Generate a graphviz dot graph from module imports'
    v1 = {
        'file_dep': ['doit.api.deps'],
        'targets': ['doit.api.dot'],
        'actions': [module_to_dot_v1],
        'clean': True,
    }
    v2 = {
        'targets': ['doit.api.dot'],
        'actions': [(module_to_dot_v2, (), {'source': 'doit.api'})],
        'getargs': {'sinks': ('imports', 'modules')},
        'clean': True,
    }
    v3 = {
        'targets': ['doit.dot'],
        'actions': [module_to_dot_v3],
        'getargs': {'imports': ('imports', 'modules')},
        'clean': True,
    }
    return v3

def task_draw():
    'Generate image from a dot file'
    v1 = {
        'file_dep': ['doit.api.dot'],
        'targets': ['doit.api.png'],
        'actions': ['dot -Tpng %(dependencies)s -o %(targets)s'],
        'clean': True,
    }
    v2 = {
        'file_dep': ['doit.dot'],
        'targets': ['doit.png'],
        'actions': ['dot -Tpng %(dependencies)s -o %(targets)s'],
        'clean': True,
    }
    return v2

def task_print():
    'print on stdout list of direct module imports'
    for name, module in PKG_MODULES.by_name.items():
        yield {
            'name': name,
            'actions': [print_imports],
            'getargs': {'modules': ('imports:{}'.format(name), 'modules')},
            'uptodate': [False],
            'verbosity': 2,
        }

def get_imports_v1(module_path):
    module = PyModule(module_path)
    base_path = module.pkg_path().resolve()
    mset = ModuleSet(base_path.glob('**/*.py'))
    imports = mset.get_imports(module, return_fqn=True)
    return {'modules': list(sorted(imports))}

def get_imports_v2(pkg_modules, module_path):
    module = pkg_modules.by_path[module_path]
    imports = pkg_modules.get_imports(module, return_fqn=True)
    return {'modules': list(sorted(imports))}

def module_to_dot_v1(dependencies, targets):
    'Make a dot file from the '
    graph = gviz.AGraph(strict=False, directed=True)
    graph.node_attr['color'] = 'lightblue2'
    graph.node_attr['style'] = 'filled'
    for dep in dependencies:
        filepath = pathlib.Path(dep)
        source = filepath.stem
        with filepath.open() as fh:
            for line in fh:
                sink = line.strip()
                if sink:
                    graph.add_edge(source, sink)
    graph.write(targets[0])

def module_to_dot_v2(source, sinks, targets):
    graph = gviz.AGraph(strict=False, directed=True)
    graph.node_attr['color'] = 'lightblue2'
    graph.node_attr['style'] = 'filled'
    for sink in sinks:
        graph.add_edge(source, sink)
    graph.write(targets[0])

def module_to_dot_v3(imports, targets):
    graph = gviz.AGraph(strict=False, directed=True)
    graph.node_attr['color'] = 'lightblue2'
    graph.node_attr['style'] = 'filled'
    for source, sinks in imports.items():
        for sink in sinks:
            graph.add_edge(source, sink)
    graph.write(targets[0])

def print_imports(modules):
    print('\n'.join(modules))
