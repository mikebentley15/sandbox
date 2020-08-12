#!/usr/bin/env python3
import two_way_dict

import unittest

class TwoWayTestCase_v1(unittest.TestCase):
    TwoWayDict = two_way_dict.TwoWayDict_v1

    def test_assignment(self):
        d = self.TwoWayDict()
        d[3] = 8
        self.assertEqual(d, {3: 8, 8: 3})
        d[7] = 6
        self.assertEqual(d, {3: 8, 8: 3, 7: 6, 6: 7})
        d[6] = 2
        self.assertEqual(d, {3: 8, 8: 3, 6: 2, 2: 6})

    def test_deletion(self):
        d = self.TwoWayDict()
        d[3] = 8
        self.assertEqual(d, {3: 8, 8: 3})
        del d[3]
        self.assertEqual(d, {})

    def test_update(self):
        d = self.TwoWayDict()
        d[3] = 8
        self.assertEqual(d, {3: 8, 8: 3})
        d.update({9: 7, 8: 2})
        self.assertEqual(d, {2: 8, 8: 2, 9: 7, 7: 9})

    def test_constructor(self):
        d = self.TwoWayDict({3: 8, 7: 6})
        self.assertEqual(d, {3: 8, 8: 3, 7: 6, 6: 7})

    def test_pop(self):
        d = self.TwoWayDict()
        d[9] = 7
        self.assertEqual(d, {9: 7, 7: 9})
        self.assertEqual(d.pop(9), 7)
        self.assertEqual(d, {})

    def test_setdefault(self):
        d = self.TwoWayDict()
        self.assertEqual(d.setdefault(4, 2), 2)
        self.assertEqual(d, {4: 2, 2: 4})

class TwoWayTestCase_v2(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v2

class TwoWayTestCase_v3(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v3

class TwoWayTestCase_v4(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v4

class TwoWayTestCase_v5(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v5

class TwoWayTestCase_v6(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v6

class TwoWayTestCase_v7(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v7

class TwoWayTestCase_v8(TwoWayTestCase_v1):
    TwoWayDict = two_way_dict.TwoWayDict_v8

if __name__ == '__main__':
    unittest.main()
