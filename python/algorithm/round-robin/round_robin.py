import heapq
from dataclasses import dataclass
from functools import total_ordering


@dataclass
@total_ordering
class PriorityItem:
    """
    Internal representation for a single item within the below PriorityQueue
    """

    cost: "typing.Any"
    item: "typing.Any"

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.cost == other.cost


class PriorityQueue:
    """
    Represents a priority queue with a custom cost function.
    """

    def __init__(self, costfunc, items=[]):
        """
        Initializes with the items and given a costfunc as the priority key.
        """
        self.heap = [PriorityItem(costfunc(item), item) for item in items]
        self.costfunc = costfunc
        heapq.heapify(self.heap)

    def pop(self):
        """
        Returns the lowest-cost element
        """
        e = heapq.heappop(self.heap)
        return e.item

    def push(self, item):
        """
        Pushes an item to the queue, with the cost computed from self.costfunc
        """
        heapq.heappush(self.heap, PriorityItem(self.costfunc(item), item))

    def is_empty(self):
        return not self

    def __bool__(self):
        return bool(self.heap)

    def __len__(self):
        return len(self.heap)


def select_nsmallest(n, items, costfunc):
    """
    Version 1: a single cost function, no duplicate detection
    """
    q = PriorityQueue(costfunc, items)
    while q and n > 0:
        n -= 1
        yield q.pop()


def select_nsmallest_without_duplicates(n, items, costfunc, is_duplicate):
    """
    Version 2: a single cost function, with duplicate detection
    """
    q = PriorityQueue(costfunc, items)
    visited = set()
    duplicates = set()
    while q and len(visited) < n:
        e = q.pop()
        if e in visited or e in duplicates:
            continue
        if is_duplicate(visited, e):
            duplicates.add(e)
            continue
        visited.add(e)
        yield e


def round_robin(n, items, costfuncs):
    """
    Version 3: multiple cost functions in a round-robin, no duplicate detection
    """
    queues = [PriorityQueue(costfunc, items) for costfunc in costfuncs]
    visited = set()
    while queues and len(visited) < n:
        for q in queues[:]:  # note: temporary copy of queues to enable modification inside loop
            e = q.pop()
            # note: if e in visited, then this round-robin slot is already satisfied
            if e not in visited:
                visited.add(e)
                yield e
            if q.is_empty():
                return  # all elements have been visited


def round_robin_without_duplicates(n, items, costfuncs, is_duplicate):
    """
    Version 4: multiple cost functions in a round-robin, with duplicate detection
    """
    queues = [PriorityQueue(costfunc, items) for costfunc in costfuncs]
    visited = set()
    duplicates = set()
    while queues and len(visited) < n:
        for q in queues[:]:  # note: temporary copy of queues to enable modification inside loop
            while q:
                e = q.pop()
                if e in visited:
                    break  # skip, this round-robin slot is already satisfied
                if e in duplicates:
                    continue  # find the next element for this round-robin slot
                if is_duplicate(visited, e):
                    duplicates.add(e)
                    continue  # find the next element for this round-robin slot
                visited.add(e)  # else, element is new
                yield e
                break
            if q.is_empty():
                return  # done, all elements are either visited or duplicate
