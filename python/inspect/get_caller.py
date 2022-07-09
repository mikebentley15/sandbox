import inspect

def get_caller_info(levels=1):
    'Return the name of the module that calls this function'
    caller_info = inspect.stack(context=0)[levels]
    return caller_info

def get_caller_file(levels=1):
    return get_caller_info(levels=levels+1).filename

def call_func(f):
    return f()
