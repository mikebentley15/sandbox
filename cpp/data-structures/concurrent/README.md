I'm studying concurrent data structures.  The easy thing to do is simply wrap an existing non-thread-safe data structure and use a mutex to only allow reading and writing from a single thread at a time.  However, there are more interesting approaches with mutexes, and with lock-free and wait-free data structure algorithms.  This folder is to explore this more deeply.

## Linked List Style Containers

Many containers can be implemented with nodes that point to each other in a linked-list style.

### Stack

Single directional unbounded stack where you can push to the top and pop from the top

### Queue

Single directional unbounded queue where you can push to one end and pop from the other

### Dequeue

Double directional unbounded queue where you can push and pop on both ends.  This can be used to implement a Stack or a Queue, with additional overhead to enable both.

### Linked List

A generic linked list datastructure where you can add and remove from both ends as well as iterate through the list.  You are also allowed to insert or remove from anywhere in the list.

## Array/Buffer Style Containers

Buffer-style containers have a predetermined capacity where they will fail to add if they reach that capacity.  This can be implemented as blocking or non-blocking data structures.

### 
