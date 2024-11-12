#ifndef LockFreeQueue_H
#define LockFreeQueue_H

#include <atomic>
#include <stdexcept>  // for std::out_of_range

#include <cassert>    // for assert()

/** Lock-free queue implementation inspired by Michael et al.
 *   From "Simple, Fast, and Practical Non-Blocking and Blocking Concurrent
 *   Queue Algorithms," by Maged M. Michael and Michael L. Scott.
 * However, this is modified by myself to fix the problems I saw and make more robust
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
class LockFreeNodeQueue {
public:
  struct Node {
    Node* next;
    T value;
    std::atomic<bool> ready_for_pop;
    Node() = default;
    explicit Node(T &&val)
      : next(nullptr)
      , value(std::move(val))
      , ready_for_pop(false)
    {}
  };

public:
  LockFreeNodeQueue()
    : _head(new Node()), _tail(_head.load()) {}

  explicit LockFreeNodeQueue(Node *initial_dummy)
    : _head(initial_dummy), _tail(initial_dummy) {}

  ~LockFreeNodeQueue() noexcept {
    Node *node = _head.load(); //std::memory_order_relaxed);
    _head.store(nullptr); //, std::memory_order_relaxed);
    while (node) {
      auto prev = node;
      node = node->next;
      delete prev;
    }
    _tail.store(nullptr); //, std::memory_order_relaxed);
  }

  /** Try to push a node onto the queue.
   *
   * @param node: a pointer to a node with a populated value.  This class then
   *    takes ownership of this pointer and will delete it if it is still
   *    within the container when the container is destroyed.
   * @return true if the node was able to be added, false otherwise.  If false
   *    is returned, then the ownership of the node's memory is not transferred
   *    to this queue class.
   */
  bool try_push(Node* node) {
    prep_node_for_push(node);
    auto cur_tail = _tail.load(); //std::memory_order_relaxed);
    return try_push_impl(node, cur_tail);
  }

  /// Same as try_push() except will loop until success, guaranteeing insertion.
  void push(Node* node) {
    prep_node_for_push(node);
    auto cur_tail = _tail.load(); //std::memory_order_relaxed);
    while (!try_push_impl(node, cur_tail)) {}
  }

  /** Try to pop off the queue
   *
   * @param[out] popped: The node that was popped off.  Set to nullptr if the
   *    queue is empty.  Note, if this pointer is non-null, it is the caller's
   *    responsibility to free it when done (or reuse it for a push()).
   * @return True if popped was set (nullptr if queue is empty, non-null if success)
   */
  bool try_pop(Node* &popped) {
    auto cur_head = _head.load(); //std::memory_order_relaxed);
    auto cur_tail = _tail.load(); //std::memory_order_relaxed);
    return try_pop_impl(popped, cur_head, cur_tail);
  }

  /** Same as try_pop except will loop until success.
   * 
   * A return value is used instead of an out parameter.  The ownership of the
   * memory with the returned Node is the caller's responsibility to free.
   */
  Node* pop() {
    Node* popped;
    auto cur_head = _head.load(); //std::memory_order_relaxed);
    auto cur_tail = _tail.load(); //std::memory_order_relaxed);
    while (!try_pop_impl(popped, cur_head, cur_tail)) {}
    return popped;
  }

private:
  void prep_node_for_push(Node* node) {
    node->next = nullptr;
    node->ready_for_pop.store(false); //, std::memory_order_release);
  }

  bool try_push_impl(Node* node, Node* &cur_tail) {
    if (cur_tail->ready_for_pop.load()) { //std::memory_order_acquire)) {
      assert(cur_tail->next);
      if (_tail.compare_exchange_weak(cur_tail, cur_tail->next))//,
                                      //std::memory_order_seq_cst))
      {
        cur_tail = _tail.load(); //std::memory_order_relaxed);
      }
    } else {
      if (_tail.compare_exchange_strong(cur_tail, node)) //, std::memory_order_seq_cst))
      {
        assert(!cur_tail->ready_for_pop.load());
        cur_tail->next = node;
        cur_tail->value = node->value;
        cur_tail->ready_for_pop.store(true); //, std::memory_order_release);
        return true;
      }
    }
    return false;
  }

  bool try_pop_impl(Node* &popped, Node* &cur_head, Node* &cur_tail) {
    assert(cur_head != nullptr);
    assert(cur_tail != nullptr);
    if (cur_head == cur_tail) {
      if (!cur_head->ready_for_pop.load()) { //std::memory_order_acquire)) {
        popped = nullptr;
        return true;
      }
      assert(cur_tail->next);
      if (_tail.compare_exchange_weak(cur_tail, cur_tail->next))//,
                                      //std::memory_order_seq_cst))
      {
        cur_tail = _tail.load(); //std::memory_order_relaxed);
      }
    } else if (cur_head->ready_for_pop.load()) { //std::memory_order_acquire)) {
      if (_head.compare_exchange_weak(cur_head, cur_head->next)) //,
                                      //std::memory_order_seq_cst))
      {
        popped = cur_head;
        return true;
      }
    }
    return false;
  }

private:
  std::atomic<Node*> _head;
  std::atomic<Node*> _tail;
};


/// The real lock-free queue of T (uses LockFreeNodeQueue internally)
template <typename T>
class LockFreeQueue {
private:
  using Node = typename LockFreeNodeQueue<T>::Node;

public:
  LockFreeQueue() : _q(new Node(T())) {}

  // TODO: maintain a queue of removed nodes for reuse

  /// Adds to the end of the queue concurrently
  void push(T element) { this->emplace(std::move(element)); }

  template <class... Args>
  void emplace(Args&&... args) {
    auto node = get_node();
    node->value = T(std::forward<Args...>(args...));
    _q.push(node);
  }

  bool try_push(T element) {
    auto node = get_node();
    node->value = std::move(element);
    bool success = _q.try_push(node);
    if (!success) { handle_removed(node); }
    return success;
  }

  bool try_pop(T &val_out) {
    Node *removed;
    bool success = _q.try_pop(removed);
    return success && handle_removed(removed, val_out);
  }

  bool pop(T &val_out) {
    auto removed = _q.pop();
    return handle_removed(removed, val_out);
  }

private:
  bool handle_removed(Node *removed, T &val_out) {
    if (removed) {
      val_out = std::move(removed->value);
      _recycling_bin.push(removed);
      return true;
    }
    return false;
  }

  void recycle(Node *removed) {
    if (removed) {
      _recycling_bin.push(removed);
    }
  }

  Node* get_node() {
    Node* node = _recycling_bin.pop();
    if (!node) {
      node = new Node(T());
    }
    return node;
  }

private:
  LockFreeNodeQueue<T> _q;             ///< underlying queue of nodes
  LockFreeNodeQueue<T> _recycling_bin; ///< to store removed nodes for later reuse
};

#endif // LockFreeQueue_H
