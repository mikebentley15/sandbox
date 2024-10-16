import unittest

import round_robin as rr

items = [
    (1, 2, 3, 4, "mike"),
    (2, 3, 4, 1, "mike"),  #
    (3, 4, 1, 2, "harry"),
    (4, 1, 2, 3, "sam"),
]
costfuncs = [
    lambda x: x[0],
    lambda x: x[1],
]


def is_duplicate(item_set, new_item):
    return any(new_item[-1] == item[-1] for item in item_set)


class TestRoundRobin(unittest.TestCase):
    def test_select_nsmallest(self):
        n = 2
        self.assertEqual(list(rr.select_nsmallest(n, items, costfuncs[0])), sorted(items, key=costfuncs[0])[:n])

    def test_select_nsmallest_without_duplicates(self):
        n = 2
        self.assertEqual(
            list(rr.select_nsmallest_without_duplicates(n, items, costfuncs[0], is_duplicate)), [items[0], items[2]]
        )

    def test_round_robin(self):
        n = 3
        self.assertEqual(list(rr.round_robin(n, items, costfuncs)), [items[0], items[3], items[1]])

    def test_round_robin_without_duplicates(self):
        n = 3
        self.assertEqual(
            list(rr.round_robin_without_duplicates(n, items, costfuncs, is_duplicate)), [items[0], items[3], items[2]]
        )


if __name__ == "__main__":
    unittest.main()
