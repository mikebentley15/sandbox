#ifndef EVENT_LOOP_H
#define EVENT_LOOP_H

struct RegisteredEventBase;

struct Event {
  // callback function.  Note, it is safe to call EventLoop.remove_event(event)
  // within this callback if it should not be called anymore.
  using Callback = void(Event*);

  Callback *callback;         // callback function
  int repeats;                // event repeats.  negative means forever
  uint32_t period_micros;     // microseconds between triggers
  unsigned long last_trigger; // last time this was triggered (from micros())
  bool auto_delete;           // delete automatically when it's time to be removed

  Event(void) {
    repeats = -1;
    period_micros = 0;
    last_trigger = micros();
    auto_delete = false;
  }
};

template <uint16_t NUM_EVENTS = 50>
class EventLoop {
public:
  EventLoop(void) {
    for (uint16_t i = 0; i < NUM_EVENTS; ++i) {
      registered_events[i] = nullptr;
    }
  }

  bool contains(Event* event) {
    if (event == nullptr) { return false; }
    return (NUM_EVENTS != this->find_event(event));
  }

  /** Creates an event and registers it.
   *
   * If successful returns the event, otherwise returns nullptr.
   *
   * @param period_micros: microseconds between triggers
   * @param callback: callback function
   * @param repeats: how many times to trigger (negative means forever)
   **/
  Event* schedule(
      uint32_t period_micros,
      Event::Callback *callback,
      int16_t repeats = -1)
  {
    // find the next empty spot
    auto first_empty = find_event(nullptr);
    if (first_empty == NUM_EVENTS) {
      return nullptr;
    }

    // create the event
    Event *event = new Event();
    event->callback = callback;
    event->repeats = repeats;
    event->period_micros = period_micros;
    event->repeats = repeats;
    event->last_trigger = micros(); // now
    event->auto_delete = true;

    // register the event
    registered_events[first_empty] = event;

    return event;
  }

  Event* schedule_frequency(
      uint32_t frequency, // Hz
      Event::Callback *callback,
      int16_t repeats = -1)
  {
    uint32_t period_micros = 1000000 / frequency;
    return schedule(period_micros, callback, repeats);
  }

  // call callback only once after 'after' time
  Event* once(Event::Callback *callback, unsigned long after) {
    return schedule(after, callback, 1);
  }

  /// Register the event.  Return true if successful (otherwise we're full)
  bool add_event(Event *event) {
    auto empty_idx = find_event(nullptr); // find the next empty event
    if (empty_idx < NUM_EVENTS) {
      registered_events[empty_idx] = event;
      return true;
    } else {
      return false; // full
    }
  }

  // remove event from event loop
  bool remove_event(Event *event) {
    auto idx = find_event(event);
    if (idx < NUM_EVENTS) {
      return remove_event(idx);
    }
    return false;
  }

  /** Runs one iteration of the event loop.
   *
   * Check the times and call events.  Returns the number of triggered events.
   */
  void update(void) {
    for (uint16_t i = 0; i < NUM_EVENTS; ++i) {
      if (registered_events[i] != nullptr) {
        this->update(i);
      }
    }
  }

  /** Resets all repeating events to now
   *
   * Specifically sets last_triggered() to now
   *
   * Useful after doing a long blocking thing so that the events do not try to
   * aggressively "catch up".
   */
  void reset(void) {
    for (uint16_t i = 0; i < NUM_EVENTS; ++i) {
      auto event = registered_events[i];
      if (event != nullptr && event->repeats != 0) {
        event->last_trigger = micros(); // now
      }
    }
  }

private:
  uint16_t find_event(Event *event) {
    for (uint16_t i = 0; i < NUM_EVENTS; ++i) {
      if (registered_events[i] == event) {
        return i;
      }
    }
    return NUM_EVENTS;
  }

  bool is_valid_idx(uint16_t idx) {
    return idx < NUM_EVENTS;
  }

  bool remove_event(uint16_t idx) {
    // check for valid index
    if (!is_valid_idx(idx)) {
      return false;
    }

    Event *event = registered_events[idx];
    if (event == nullptr) {
      return false;
    }
    if (event->auto_delete) {
      delete event;
    }
    registered_events[idx] = nullptr;
    return true;
  }

  // Content in the event loop
  void update(uint16_t idx) {
    // check for valid index
    if (!is_valid_idx(idx)) {
      return;
    }

    // check for valid event
    Event *event = registered_events[idx];
    if (event == nullptr) {
      return;
    }

    // check if the event triggers on timing
    unsigned long now = micros();
    unsigned long delta = now - event->last_trigger;
    if (delta >= event->period_micros) {
      // rather than when it was really triggered, mark when it ideally should
      // have been triggered.  This is to allow the next one to "catch up" if
      // this one was late.
      // However, if it is late by more than the period, truncate to being one
      // period behind.
      unsigned long offset = min(event->period_micros,
                                 delta - event->period_micros);
      event->last_trigger = now - offset;

      // make it safe for the callback to call remove_event(
      event->callback(event);
      if (event == registered_events[idx]) { // it's still there
        if (event->repeats > 0) {
          event->repeats -= 1;
        }
        if (event->repeats == 0) {
          remove_event(idx);
        }
      }
    }
  }

private:
  Event* registered_events[NUM_EVENTS]; // event array
};

#endif // EVENT_LOOP_H
