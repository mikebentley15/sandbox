#ifndef LockFreeQueue_MMMS_H
#define LockFreeQueue_MMMS_H

#include <atomic>
#include <stdexcept>  // for std::out_of_range

/** Lock-free queue implementation from Michael et al.
 *   From "Simple, Fast, and Practical Non-Blocking and Blocking Concurrent
 *   Queue Algorithms," by Maged M. Michael and Michael L. Scott.
 *
 * Invariants:
 *   1. The linked list is always connected.
 *   2. Nodes are only inserted after the last node in the linked list.
 *   3. Nodes are only deleted from the beginning of the linked list.
 *   4. Head always points to the node before the first in the linked list.
 *   5. Tail always points to a node in the linked list.
 *
 * One change from their implementation is the removal of the count used in the
 * compare_exchange calls.  The reason this was added was to make the ABA
 * problem less likely to occur.  If the count is used again, it makes reusing
 * removed nodes easier to do.  You could additionally keep free nodes in some
 * other data structure, like another concurrent stack or queue (where nodes
 * are given rather than allocated).
 */
template <typename T>
class LockFreeQueue_MMMS {
public:
  LockFreeQueue_MMMS() : _q(new Node(T())) {}

  // TODO: maintain a queue of removed nodes for reuse

  /// Adds to the end of the queue concurrently
  void push(T element) { this->emplace(std::move(element)); }
  void emplace(T &&element) {
    auto node = new Node(std::move(element));
    _q.push(node);
  }

  /** Removes the next available item from the queue.
   *
   * If the queue is empty, then this throws std::out_of_range().
   */
  bool pop(T &val_out) {
    auto removed = _q.pop(val_out);
    delete removed;
    return removed != nullptr;
  }

private:
  struct Node {
    T value;
    std::atomic<Node*> next;
    explicit Node(T &&val) : value(std::move(val)), next(nullptr) {}
  };

  /// Maintains a queue of nodes instead of a queue of values
  struct NodeQueue {
    std::atomic<Node*> head;
    std::atomic<Node*> tail;
    explicit NodeQueue(Node *initial_dummy)
      : head(initial_dummy), tail(initial_dummy) {}
    ~NodeQueue() noexcept {
      Node *node;
      while (nullptr != (node = head.load(std::memory_order_relaxed))) {
        auto next = node->next.load(std::memory_order_relaxed);
        head.store(next, std::memory_order_relaxed);
        delete node;
      }
    }

    void push(Node *node) {
      node->next.store(nullptr, std::memory_order_relaxed);
      Node* cur_tail = nullptr;
      for (;;) {
        cur_tail = tail.load(std::memory_order_relaxed);
        auto next = cur_tail->next.load(std::memory_order_relaxed);
        if (cur_tail == tail.load(std::memory_order_relaxed)) {
          if (next == nullptr) {
            if (cur_tail->next.compare_exchange_weak(next, node)) {
              break;
            }
          } else {
            tail.compare_exchange_weak(cur_tail, next);
          }
        }
      }
      tail.compare_exchange_strong(cur_tail, node);
    }

    Node* pop(T &val_out) {
      Node *cur_head = nullptr;
      for (;;) {
        cur_head = head.load(std::memory_order_relaxed);
        auto cur_tail = tail.load(std::memory_order_relaxed);
        auto next = cur_head->next.load(std::memory_order_relaxed);
        if (cur_head == head.load(std::memory_order_relaxed)) {
          if (cur_head == cur_tail) {
            if (next == nullptr) {
              return nullptr;
            }
            tail.compare_exchange_strong(cur_tail, next);
          } else {
            // FIXME: the next node may be deleted by this point
            val_out = next->value;
            if (head.compare_exchange_weak(cur_head, next)) {
              break;
            }
          }
        }
      }
      return cur_head;
    }
  }; // end of struct NodeQueue

private:
  NodeQueue _q;
  // NodeQueue _recycling_bin; // to store removed nodes for later reuse
};

#endif // LockFreeQueue_H
