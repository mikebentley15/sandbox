from collections import Counter

class Pmf(Counter):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        delattr(self, 'elements')

    def __truediv__(self, divisor):
        p = Pmf()
        for key, value in self.items():
            p[key] = value / divisor
        return p
