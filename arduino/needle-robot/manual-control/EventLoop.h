#ifndef EVENT_LOOP_H
#define EVENT_LOOP_H

struct RegisteredEventBase;

// callback function type
// if callback returns true, will remove the registered event
// if it returns false, then keep the registered event for later
using callback_t = bool (*)(RegisteredEventBase*);

struct RegisteredEventBase {
  callback_t callback;        // callback function
  int repeats;                // event repeats.  negative means forever
  uint32_t period_micros;     // microseconds between triggers
  unsigned long last_trigger; // last time this was triggered (from micros())
  bool auto_delete;           // delete automatically when it's time to be removed

  RegisteredEventBase(void) {
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
    for (int i = 0; i < NUM_EVENTS; ++i) {
      registered_events[i] = nullptr;
    }
  }

  /** Creates an event and registers it.
   *
   * If successful returns the event, otherwise returns nullptr.
   *
   * @param period_micros: microseconds between triggers
   * @param callback: callback function
   * @param repeats: how many times to trigger (negative means forever)
   **/
  RegisteredEventBase* schedule(
      uint32_t period_micros,
      callback_t callback,
      int16_t repeats = -1)
  {
    // find the next empty spot
    int first_empty = find_event(nullptr);
    if (first_empty == NUM_EVENTS) {
      return nullptr;
    }

    // create the event
    RegisteredEventBase *event = new RegisteredEventBase();
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

  RegisteredEventBase* schedule_frequency(
      uint32_t frequency, // Hz
      callback_t callback,
      int16_t repeats = -1)
  {
    uint32_t period_micros = 1000000 / frequency;
    return schedule(period_micros, callback, repeats);
  }

  // call callback only once after 'after' time
  RegisteredEventBase* once(callback_t callback, unsigned long after) {
    return schedule(after, callback, 1);
  }

  /// Register the event.  Return true if successful (otherwise we're full)
  bool add_event(RegisteredEventBase *event) {
    int empty_idx = find_event(nullptr); // find the next empty event
    if (empty_idx < NUM_EVENTS) {
      registered_events[empty_idx] = event;
      return true;
    } else {
      return false; // full
    }
  }

  // remove event from event loop
  bool remove_event(RegisteredEventBase *event) {
    int idx = find_event(event);
    if (idx < NUM_EVENTS) {
      return remove_event(idx);
    }
    return false;
  }

  /** Runs one iteration of the event loop.
   *
   * Check the times and call events.  Returns the number of triggered events.
   */
  uint16_t update(void) {
    uint16_t count = 0;
    for (int16_t i = 0; i < NUM_EVENTS; ++i) {
      if (registered_events[i] != nullptr) {
        count += (update(i) == true) ? 1 : 0;
      }
    }
    return count;
  }

private:
  int find_event(RegisteredEventBase *event) {
    for (int i = 0; i < NUM_EVENTS; ++i) {
      if (registered_events[i] == event) {
        return i;
      }
    }
    return NUM_EVENTS;
  }

  bool is_valid_idx(int16_t idx) {
    return 0 <= idx && idx < NUM_EVENTS;
  }

  bool remove_event(int16_t idx) {
    // check for valid index
    if (!is_valid_idx(idx)) {
      return false;
    }

    RegisteredEventBase *event = registered_events[idx];
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
  bool update(int16_t idx) {
    // check for valid index
    if (!is_valid_idx(idx)) {
      return false;
    }

    // check for valid event
    RegisteredEventBase *event = registered_events[idx];
    if (event == nullptr) {
      return false;
    }

    // check if the event triggers on timing
    unsigned long now = micros();
    unsigned long delta = now - event->last_trigger;
    if (delta >= event->period_micros) {
      event->last_trigger = now - delta + event->period_micros;
      if ((event->callback)(event)) { // call callback
        remove_event(idx);
      } else {
        if (event->repeats > 0) {
          event->repeats -= 1;
        }
        if (event->repeats == 0) {
          remove_event(idx);
        }
      }
      return true; // triggered
    }

    // else, no triggering
    return false;
  }

private:
  RegisteredEventBase* registered_events[NUM_EVENTS]; // event array
};

#endif // EVENT_LOOP_H
