class TwoWayDict_v1(dict): pass

class TwoWayDict_v2(TwoWayDict_v1):

    def __delitem__(self, key):
        value = super().pop(key)
        super().pop(value, None)

    def __setitem__(self, key, value):
        if key in self:
            del self[self[key]]
        if value in self:
            del self[value]
        super().__setitem__(key, value)
        super().__setitem__(value, key)

    def __repr__(self):
        return f'{type(self).__name__}({super().__repr__()})'

class TwoWayDict_v3(TwoWayDict_v2):
    def update(self, items):
        if isinstance(items, dict):
            items = items.items()
        for key, value in items:
            self[key] = value

class TwoWayDict_v4(TwoWayDict_v3):
    def __init__(self, items=()):
        self.update(items)

DEFAULT= object()

class TwoWayDict_v5(TwoWayDict_v4):
    def pop(self, key, default=DEFAULT):
        if key in self or default is DEFAULT:
            value = self[key]
            del self[key]
            return value
        else:
            return default

class TwoWayDict_v6(TwoWayDict_v5):
    def setdefault(self, key, value):
        if key not in self:
            self[key] = value
        return self[key]

from collections.abc import MutableMapping

class TwoWayDict_v7(MutableMapping):

    def __init__(self, data=()):
        self.mapping = {}
        self.update(data)

    def __getitem__(self, key):
        return self.mapping[key]

    def __delitem__(self, key):
        value = self[key]
        del self.mapping[key]
        self.pop(value, None)

    def __setitem__(self, key, value):
        if key in self:
            del self[self[key]]
        if value in self:
            del self[value]
        self.mapping[key] = value
        self.mapping[value] = key

    def __iter__(self):
        return iter(self.mapping)

    def __len__(self):
        return len(self.mapping)

    def __repr__(self):
        return f'{type(self).__name__}({self.mapping})'

from collections import UserDict

class TwoWayDict_v8(UserDict):

    def __delitem__(self, key):
        value = self.data.pop(key)
        self.data.pop(value, None)

    def __setitem__(self, key, value):
        if key in self:
            del self[self[key]]
        if value in self:
            del self[value]
        self.data[key] = value
        self.data[value] = key

    def __repr__(self):
        return f'{type(self).__name__}({self.data})'
