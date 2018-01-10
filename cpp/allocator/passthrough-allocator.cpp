#include <memory>
#include <unordered_set>
#include <iostream>
#include <iomanip>
#include <limits>

#include <cstdint>

#define UNUSED_ARG(x) (void)x
#define ALLOCATE_SIZE 40000

template<typename T>
class Allocator {
public : 
  //    typedefs

  typedef T                   value_type;
  typedef value_type*         pointer;
  typedef const value_type*   const_pointer;
  typedef value_type&         reference;
  typedef const value_type&   const_reference;
  typedef std::size_t         size_type;
  typedef std::ptrdiff_t      difference_type;

public : 
  //    convert an allocator<T> to allocator<U>

  template<typename U>
  struct rebind {
      typedef Allocator<U> other;
  };

public : 
  inline explicit Allocator() {}
  inline ~Allocator() {}
  inline explicit Allocator(Allocator const&) {}
  template<typename U>
  inline explicit Allocator(Allocator<U> const&) {}

  //    address

  inline pointer address(reference r) { return &r; }
  inline const_pointer address(const_reference r) { return &r; }

  //    memory allocation

  inline pointer allocate(size_type cnt, 
     typename std::allocator<void>::const_pointer = 0) { 
    return reinterpret_cast<pointer>(::operator new(cnt * sizeof (T))); 
  }
  inline void deallocate(pointer p, size_type) { 
    ::operator delete(p); 
  }

  //    size

  inline size_type max_size() const { 
    return std::numeric_limits<size_type>::max() / sizeof(T);
  } 

  //    construction/destruction

  inline void construct(pointer p, const T& t) { new(p) T(t); }
  inline void destroy(pointer p) { p->~T(); }

  inline bool operator==(Allocator const&) { return true; }
  inline bool operator!=(Allocator const& a) { return !operator==(a); }
}; // end of class Allocator


template<typename T>
class AllocatorBase {
public : 
  //    typedefs

  typedef T                   value_type;
  typedef value_type*         pointer;
  typedef const value_type*   const_pointer;
  typedef value_type&         reference;
  typedef const value_type&   const_reference;
  typedef std::size_t         size_type;
  typedef std::ptrdiff_t      difference_type;

public : 
  //    convert an allocator<T> to allocator<U>

public : 
  inline explicit AllocatorBase() {}
  inline ~AllocatorBase() {}
  inline explicit AllocatorBase(AllocatorBase const&) {}
  template<typename U>
  inline explicit AllocatorBase(AllocatorBase<U> const&) {}

  //    address

  inline pointer address(reference r) { return &r; }
  inline const_pointer address(const_reference r) { return &r; }

  //    memory allocation

  //inline pointer allocate(size_type cnt, 
  //   typename std::allocator<void>::const_pointer = 0) { 
  //  return reinterpret_cast<pointer>(::operator new(cnt * sizeof (T))); 
  //}
  //inline void deallocate(pointer p, size_type) { 
  //    ::operator delete(p); 
  //}

  //    size

  inline size_type max_size() const { 
    return std::numeric_limits<size_type>::max() / sizeof(T);
  } 

  //    construction/destruction

  inline void construct(pointer p, const T& t) { new(p) T(t); }
  inline void destroy(pointer p) { p->~T(); }

  inline bool operator==(AllocatorBase const&) { return true; }
  inline bool operator!=(AllocatorBase const& a) { return !operator==(a); }
}; // end of class AllocatorBase 


//class PassthroughAllocator : public AllocatorBase {
//public:
//  typedef uint64_t          value_type;
//  typedef value_type*       pointer;
//  typedef const value_type* const_pointer;
//  typedef value_type&       reference;
//  typedef const value_type& const_reference;
//  typedef std::size_t       size_type;
//  typedef std::ptrdiff_t    difference_type;
//
//public:
//  PassthroughAllocator() noexcept {}
//
//  uint64_t* allocate (std::size_t n) const noexcept {
//    std::cout << "PassthroughAllocator::allocate(" << n << ")" << std::endl;
//    return base.allocate (n);
//  }
//
//  void deallocate (uint64_t* p, std::size_t n) const noexcept {
//    std::cout << "PassthroughAllocator::deallocate(0x"
//              << std::hex << reinterpret_cast<uint64_t>(p)
//              << ", "
//              << std::dec << n
//              << ")" << std::endl;
//    base.deallocate (p, n);
//  }
//
//private:
//  mutable std::allocator<uint64_t> base;
//};

/// First call to allocate is remembered, allocated, and kept
/// All subsequent calls use this remembered one.
template<typename T>
class MemoryAllocator {
public:
  typedef T                   value_type;
  typedef value_type*         pointer;
  typedef const value_type*   const_pointer;
  typedef value_type&         reference;
  typedef const value_type&   const_reference;
  typedef std::size_t         size_type;
  typedef std::ptrdiff_t      difference_type;

  template<typename U>
  struct rebind {
      typedef MemoryAllocator<U> other;
  };

public:
  MemoryAllocator() noexcept : data(nullptr) {
    std::cout << "MemoryAllocator::constructor()" << std::endl;
  }

  inline ~MemoryAllocator() {
    std::cout << "MemoryAllocator::destructor()" << std::endl;
    if (data != nullptr) {
      std::cout << "MemoryAllocator::destructor(): calling free" << std::endl;
      //free(data);
      data = nullptr;
    }
  }

  inline explicit MemoryAllocator(MemoryAllocator const& other) : data(nullptr) {
    std::cout << "MemoryAllocator::constructor(other)" << std::endl;
  }

  template<typename U>
  inline explicit MemoryAllocator(MemoryAllocator<U> const& other) : data(nullptr) {
    std::cout << "MemoryAllocator::constructor(other<U>)" << std::endl;
  }

  pointer allocate (size_type n, const void* hint=0) const noexcept {
    std::cout << "MemoryAllocator::allocate(" << n << ")" << std::endl;
    if (data == nullptr) {
      std::cout << "MemoryAllocator::allocate(): allocating for the first time" << std::endl;
      //data = static_cast<pointer>(malloc(n * sizeof(value_type)));
      data = reinterpret_cast<pointer>(::new void*[n * sizeof(uint64_t)]);
    }
    return data;
  }

  void deallocate (pointer p, size_type n) const noexcept {
    std::cout << "MemoryAllocator::deallocate(0x"
              << std::hex << reinterpret_cast<uint64_t>(p)
              << ", "
              << std::dec << n
              << ")" << std::endl;
  }

private:
  mutable pointer data;
};

//class SimpleAllocator : public AllocatorBase {
//public:
//  typedef uint64_t          value_type;
//  typedef value_type*       pointer;
//  typedef const value_type* const_pointer;
//  typedef value_type&       reference;
//  typedef const value_type& const_reference;
//  typedef std::size_t       size_type;
//  typedef std::ptrdiff_t    difference_type;
//
//public:
//  SimpleAllocator() noexcept {}
//
//  uint64_t* allocate (std::size_t n) {
//    std::cout << "SimpleAllocator::allocate(" << n << ")" << std::endl;
//    return data;
//  }
//
//  uint64_t* deallocate (uint64_t* p, std::size_t n) {
//    std::cout << "SimpleAllocator::deallocate(0x"
//              << std::hex << reinterpret_cast<uint64_t>(p)
//              << ", "
//              << std::dec << n
//              << ")" << std::endl;
//  }
//
//private:
//  uint64_t data[25000];
//};

template<class Alloc>
using UintUnorderedSet = std::unordered_set<uint64_t,
                                              std::hash<uint64_t>,
                                              std::equal_to<uint64_t>,
                                              Alloc>;

class SimpleSet {
public:
  SimpleSet(int capacity = 10)
    : _capacity(capacity*2)
    , _data(_capacity)
  {}

  void insert(uint64_t elem) {
    auto idx = p_hash(elem);
    if (!this->p_contains(idx, elem)) {
      _data[idx].push_front(elem);
    }

    bool should_insert_at_end = true;
    for (auto it = _data[idx].begin(); it != _data[idx].end(); it++) {
      if (elem == *it) {
        should_insert_at_end = false;
        break;
      } else if (elem < *it) {
        _data[idx].insert(it, elem);
      }
    }

    if (should_insert_at_end) {
      _data[idx].push_back(elem);
    }
  }

  bool contains(uint64_t elem) {
    auto idx = p_hash(elem);

    for (auto val : _data[idx]) {
      if (val == elem) {
        return true;
      } else if (val < elem) {
        break;
      }
    }

    return false;
  }

  std::vector<std::list<uint64_t>> data() { return _data; }

private:
  inline std::size_t p_hash(uint64_t val) {
    return _capacity % std::hash<uint64_t>(val);
  }

  bool p_contains(int idx, uint64_t elem) {
    for (auto val : _data[idx]) {
      if (val == 
    }
    auto& linked_list = _data[idx];
  }

  bool p_insert(int idx, uint64_t elem) {
    // Insert
  }

private:
  std::vector<std::list<uint64_t>> _data;
}

int main(int argCount, char* argList[]) {
  UNUSED_ARG(argCount);
  UNUSED_ARG(argList);

  //UintUnorderedSet<Allocator<uint64_t>> example(25000);

  //UintUnorderedSet<PassthroughAllocator> passthrough(25000);
  std::cout << "Before set construction" << std::endl << std::endl;
  UintUnorderedSet<MemoryAllocator<uint64_t>>      memory(25000);
  //UintUnorderedSet<SimpleAllocator>      simple(25000);

  std::cout << std::endl
            << "Before inserting 1, 2, 3" << std::endl << std::endl;
  memory.insert(1);
  memory.insert(2);
  memory.insert(3);
  memory.insert(3);
  memory.insert(3);

  std::cout << std::endl
            << "Before clear" << std::endl << std::endl;
  memory.clear();

  std::cout << std::endl
            << "Before exiting" << std::endl << std::endl;

  //passthrough.clear();
  //memory.clear();
  //simple.clear();

  return 0;
}
