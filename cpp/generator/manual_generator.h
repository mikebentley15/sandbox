#ifndef MANUAL_GENERATOR_H
#define MANUAL_GENERATOR_H

class ManualGenerator {
public:
  class iterator {
  public:
    iterator() = default;
    iterator(ManualGenerator *gen) : _gen(gen) { }

    friend bool operator == (const iterator &a, const iterator &b) {
      return a._gen == b._gen
        || (!a._gen && b._gen->_cur >= b._gen->_stop)
        || (!b._gen && a._gen->_cur >= a._gen->_stop);
    }
  private:
    ManualGenerator *_gen;
  };
public:
  ManualGenerator(int stop)
    : _line(0), _start(0), _stop(stop), _step(1) { }
  ManualGenerator(int start, int stop, int step = 1)
    : _line(0), _start(start), _stop(stop), _step(step) { }

  iterator begin() const { return iterator(this); }
  iterator end() const { return iterator(); }
  
protected:
  uint32_t _line;
  int _start;
  int _stop;
  int _step;
};

#endif // MANUAL_GENERATOR_H
