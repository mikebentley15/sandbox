#ifndef GAMEPAD_CLASS_H
#define GAMEPAD_CLASS_H

#include "gamepad/gamepad.h"

#include <QtCore/QObject>
#include <QtCore/QTimer>

#include <unordered_map>
#include <string>
#include <vector>

// // A one to one mapping that can be referenced either from key->value or
// // value->key.  Both types are zero-based index types
// //
// // The set_mapping() operation will actually do a swap to ensure a one-to-one
// // mapping.
// //
// // Initializes to map 0 - (N-1) to 0 - (N-1), i.e., the identity mapping
// template <int N>
// class IndexMapper {
// public:
//   IndexMapper() {
//     for (int i = 0; i < N; i++) {
//       _val_to_key[i] = i;
//       _key_to_val[i] = i;
//     }
//   }
//
//   int size() const { return N; }
//
//   // TODO: add ability to iterate over (key, value) pairs.
//
//   int get_val(int key) const { return _key_to_val[key]; }
//   int get_key(int val) const { return _val_to_key[val]; }
//
//   void set_mapping(int key, int val) {
//     if (_key_to_val[key] == val) {
//       return;
//     }
//     if (key >= N || val >= N) {
//       throw std::out_of_range("key or val too big for IndexMapper");
//     }
//
//     // suppose this situation:
//     //   0 -> 3
//     //   1 -> 2
//     //   2 -> 1
//     //   3 -> 0
//     // then I call set_mapping(1, 0).  I want it to then look like
//     //   0 -> 3
//     //   1 -> 0
//     //   2 -> 1
//     //   3 -> 2
//     int oldval = _key_to_val[key];
//     int oldkey = _val_to_key[val];
//     _key_to_val[key]    = val;
//     _val_to_key[val]    = key;
//     _key_to_val[oldkey] = oldval;
//     _val_to_key[oldval] = oldkey;
//   }
//
// private:
//   int _val_to_key[N];
//   int _key_to_val[N];
// };


class GamepadManager;

class Gamepad : public QObject {
  Q_OBJECT

private:

  // TODO: add ability to remap buttons
  //using NotifySignalPtr = void (Gamepad::*)();

  //struct Button {
  //  std::string name;
  //  NotifySignalPtr pressed_signal;
  //  NotifySignalPtr released_signal;
  //};

  //static Button buttons[] = {
  //  {"hello", nullptr, nullptr},
  //};

public:
  using FloatPair = std::pair<float, float>;
  using IntPair = std::pair<int, int>;

  int id() const { return static_cast<int>(_id); };
  bool is_connected() const { return _is_connected; }
  bool is_button_pressed(int button) const {
    return GamepadButtonDown(_id, static_cast<GAMEPAD_BUTTON>(button));
  }
  bool is_trigger_pressed(int trigger) const {
    return GamepadTriggerDown(_id, static_cast<GAMEPAD_TRIGGER>(trigger));
  }
  bool is_dpad_up_pressed() const { return is_button_pressed(BUTTON_DPAD_UP); }
  bool is_dpad_down_pressed() const { return is_button_pressed(BUTTON_DPAD_DOWN); }
  bool is_dpad_left_pressed() const { return is_button_pressed(BUTTON_DPAD_LEFT); }
  bool is_dpad_right_pressed() const { return is_button_pressed(BUTTON_DPAD_RIGHT); }
  bool is_start_pressed() const { return is_button_pressed(BUTTON_START); }
  bool is_select_pressed() const { return is_button_pressed(BUTTON_BACK); }
  bool is_L1_pressed() const { return is_button_pressed(BUTTON_LEFT_SHOULDER); }
  bool is_R1_pressed() const { return is_button_pressed(BUTTON_RIGHT_SHOULDER); }
  bool is_L2_pressed() const { return is_trigger_pressed(TRIGGER_LEFT); }
  bool is_R2_pressed() const { return is_trigger_pressed(TRIGGER_RIGHT); }
  bool is_L3_pressed() const { return is_button_pressed(BUTTON_LEFT_THUMB); }
  bool is_R3_pressed() const { return is_button_pressed(BUTTON_RIGHT_THUMB); }
  bool is_X_pressed() const { return is_button_pressed(BUTTON_X); }
  bool is_Y_pressed() const { return is_button_pressed(BUTTON_Y); }
  bool is_A_pressed() const { return is_button_pressed(BUTTON_A); }
  bool is_B_pressed() const { return is_button_pressed(BUTTON_B); }

  float L2_trigger_value() const { return GamepadTriggerLength(_id, TRIGGER_LEFT); }
  float R2_trigger_value() const { return GamepadTriggerLength(_id, TRIGGER_RIGHT); }
  int L2_trigger_raw() const { return _l2_raw; } // TODO: upate with GamepadTriggerValue
  int R2_trigger_raw() const { return _r2_raw; }

  FloatPair left_joy_xy() const {
    float x, y;
    GamepadStickNormXY(_id, STICK_LEFT, &x, &y);
    return {x, y};
  }

  // pair of rotation (radians) and magnitude (max 1.0)
  FloatPair left_joy_rotmag() const {
    return {
      GamepadStickAngle(_id, STICK_LEFT),
      GamepadStickLength(_id, STICK_LEFT)
    };
  }

  FloatPair right_joy_xy() const {
    float x, y;
    GamepadStickNormXY(_id, STICK_RIGHT, &x, &y);
    return {x, y};
  }

  FloatPair right_joy_rotmag() const {
    return {
      GamepadStickAngle(_id, STICK_RIGHT),
      GamepadStickLength(_id, STICK_RIGHT)
    };
  }

  IntPair left_joy_raw() const { return {_left_joy_x, _left_joy_y}; } // TODO: update with GamepadStickXY
  IntPair right_joy_raw() const { return {_right_joy_x, _right_joy_y}; }

signals:
  void connected();
  void disconnected();
  void button_pressed(int button);
  void button_released(int button);
  void trigger_pressed(int trigger);
  void trigger_released(int trigger);
  void stick_direction_changed(int stick, int direction);

  void dpad_up_pressed();
  void dpad_up_released();
  void dpad_down_pressed();
  void dpad_down_released();
  void dpad_left_pressed();
  void dpad_left_released();
  void dpad_right_pressed();
  void dpad_right_released();
  void start_pressed();
  void start_released();
  void select_pressed();
  void select_released();
  void L1_pressed();
  void L1_released();
  void R1_pressed();
  void R1_released();
  void L2_pressed();
  void L2_released();
  void R2_pressed();
  void R2_released();
  void L3_pressed();
  void L3_released();
  void R3_pressed();
  void R3_released();
  void X_pressed();
  void X_released();
  void Y_pressed();
  void Y_released();
  void A_pressed();
  void A_released();
  void B_pressed();
  void B_released();
  void L2_value_changed(float val);
  void R2_value_changed(float val);
  void left_joystick_changed (float rotation, float magnitude);
  void right_joystick_changed(float rotation, float magnitude);

private:
  // Private constructor that only GamepadManager can call
  Gamepad(QObject *parent, int id)
    : QObject(parent)
    , _id(static_cast<GAMEPAD_DEVICE>(id))
    , _is_connected(false)
    , _l2_raw(0)
    , _r2_raw(0)
    , _left_joy_x(0)
    , _left_joy_y(0)
    , _right_joy_x(0)
    , _right_joy_y(0)
  {
    connect(this, &Gamepad::button_pressed,
            this, &Gamepad::emit_specific_button_pressed);
    connect(this, &Gamepad::button_released,
            this, &Gamepad::emit_specific_button_released);
    connect(this, &Gamepad::trigger_pressed,
            this, &Gamepad::emit_specific_trigger_pressed);
    connect(this, &Gamepad::trigger_released,
            this, &Gamepad::emit_specific_trigger_released);
    connect(this, &Gamepad::connected,
        [this]() {
          update_trigger_value(TRIGGER_LEFT, 0);
          update_trigger_value(TRIGGER_RIGHT, 0);
          update_joystick_value(STICK_LEFT, 0, 0);
          update_joystick_value(STICK_RIGHT, 0, 0);
        });
  }

  void mark_connected()    { _is_connected = true; }
  void mark_disconnected() { _is_connected = false; }

private slots:
  // TODO: emit signals for things like button_a() and so forth
  // TODO: implement a mapping for button integer to button type
  // TODO: allow the user to modify the mapping

  void emit_specific_button_pressed(int button) {
    switch (static_cast<GAMEPAD_BUTTON>(button)) {
      case BUTTON_DPAD_UP:        emit dpad_up_pressed();    break;
      case BUTTON_DPAD_DOWN:      emit dpad_down_pressed();  break;
      case BUTTON_DPAD_LEFT:      emit dpad_left_pressed();  break;
      case BUTTON_DPAD_RIGHT:     emit dpad_right_pressed(); break;
      case BUTTON_START:          emit start_pressed();      break;
      case BUTTON_BACK:           emit select_pressed();     break;
      case BUTTON_LEFT_THUMB:     emit L3_pressed();         break;
      case BUTTON_RIGHT_THUMB:    emit R3_pressed();         break;
      case BUTTON_LEFT_SHOULDER:  emit L1_pressed();         break;
      case BUTTON_RIGHT_SHOULDER: emit R1_pressed();         break;
      case BUTTON_A:              emit A_pressed();          break;
      case BUTTON_B:              emit B_pressed();          break;
      case BUTTON_X:              emit X_pressed();          break;
      case BUTTON_Y:              emit Y_pressed();          break;
      default: break; // do nothing
    }
  }

  void emit_specific_button_released(int button) {
    switch (static_cast<GAMEPAD_BUTTON>(button)) {
      case BUTTON_DPAD_UP:        emit dpad_up_released();    break;
      case BUTTON_DPAD_DOWN:      emit dpad_down_released();  break;
      case BUTTON_DPAD_LEFT:      emit dpad_left_released();  break;
      case BUTTON_DPAD_RIGHT:     emit dpad_right_released(); break;
      case BUTTON_START:          emit start_released();      break;
      case BUTTON_BACK:           emit select_released();     break;
      case BUTTON_LEFT_THUMB:     emit L3_released();         break;
      case BUTTON_RIGHT_THUMB:    emit R3_released();         break;
      case BUTTON_LEFT_SHOULDER:  emit L1_released();         break;
      case BUTTON_RIGHT_SHOULDER: emit R1_released();         break;
      case BUTTON_A:              emit A_released();          break;
      case BUTTON_B:              emit B_released();          break;
      case BUTTON_X:              emit X_released();          break;
      case BUTTON_Y:              emit Y_released();          break;
      default: break; // do nothing
    }
  }

  void emit_specific_trigger_pressed(int trigger) {
    switch (static_cast<GAMEPAD_TRIGGER>(trigger)) {
      case TRIGGER_LEFT:  emit L2_pressed(); break;
      case TRIGGER_RIGHT: emit R2_pressed(); break;
      default: break; // do nothing
    }
  }

  void emit_specific_trigger_released(int trigger) {
    switch (static_cast<GAMEPAD_TRIGGER>(trigger)) {
      case TRIGGER_LEFT:  emit L2_released(); break;
      case TRIGGER_RIGHT: emit R2_released(); break;
      default: break; // do nothing
    }
  }

  void update_trigger_value(int trigger, int value) {
    switch (trigger) {
      case TRIGGER_LEFT:
        // TODO: do I want to be less sensitive?
        if (value != _l2_raw) {
          emit L2_value_changed(L2_trigger_value());
        }
        _l2_raw = value;
        break;

      case TRIGGER_RIGHT:
        if (value != _r2_raw) {
          emit R2_value_changed(R2_trigger_value());
        }
        _r2_raw = value;
        break;

      default: break; // do nothing
    }
  }

  void update_joystick_value(int stick, int x, int y) {
    switch (stick) {
      case STICK_LEFT:
        // TODO: do I want to be less sensitive?
        if (x != _left_joy_x || y != _left_joy_y) {
          auto [rotation, magnitude] = left_joy_rotmag();
          emit left_joystick_changed(rotation, magnitude);
        }
        _left_joy_x = x;
        _left_joy_y = y;
        break;

      case STICK_RIGHT:
        if (x != _right_joy_x || y != _right_joy_y) {
          auto [rotation, magnitude] = right_joy_rotmag();
          emit right_joystick_changed(rotation, magnitude);
        }
        _right_joy_x = x;
        _right_joy_y = y;
        break;

      default: break; // do nothing
    }
  }

private:
  const GAMEPAD_DEVICE _id; // gamepad index
  bool _is_connected;       // cached connection result to allow event detection
  int _l2_raw;              // left trigger raw value last time
  int _r2_raw;              // right trigger raw value last time
  int _left_joy_x;          // left x joystick raw value last time
  int _left_joy_y;          // left y joystick raw value last time
  int _right_joy_x;         // right x joystick raw value last time
  int _right_joy_y;         // right y joystick raw value last time

private:
  friend class GamepadManager;
};

/**
 * Wraps around libgamepad to create signals on the gamepad events.
 *
 * Only one of these objects should be created at time.  It is suggested you
 * create this in your main() function similar to how you create a QApplication
 * or QCoreApplication.  After that, you can access it through the singleton
 * pointer GamepadManager::instance.
 */
class GamepadManager : public QObject {
  Q_OBJECT

  Q_PROPERTY(unsigned poll_interval READ poll_interval WRITE poll_interval)

public:
  GamepadManager(QObject *parent = nullptr)
    : QObject(parent)
    , _timer(new QTimer(this))
  {
    // initialize gamepads
    GamepadInit();
    for (int id = 0; id != GAMEPAD_COUNT; id++) {
      _gamepads.emplace_back(new Gamepad(this, id));
    }

    // setup the timer to call update_gamepads()
    poll_interval(16); // default polling is 16 ms (60 Hz)
    connect(_timer, &QTimer::timeout,
            this, &GamepadManager::update_gamepads);
    _timer->start();
  }

  ~GamepadManager() {
    GamepadShutdown();
  }

  // gamepad getters
  size_t size()                             { return _gamepads.size(); }
  Gamepad* operator[](size_t i)             { return _gamepads[i];     }
  Gamepad* at(size_t i)                     { return _gamepads.at(i);  }
  const Gamepad* operator[](size_t i) const { return _gamepads[i];     }
  const Gamepad* at(size_t i)         const { return _gamepads.at(i);  }

  // poll_millisec getter and setter
  unsigned int poll_interval() const        { return _timer->interval(); }
  void poll_interval(unsigned int millisecond_interval) {
    _timer->setInterval(millisecond_interval);
  }

signals:
  void gamepad_connected      (int gamepad_index);
  void gamepad_disconnected   (int gamepad_index);
  void button_pressed         (int button,  int gamepad_index);
  void button_released        (int button,  int gamepad_index);
  void trigger_pressed        (int trigger, int gamepad_index);
  void trigger_released       (int trigger, int gamepad_index);
  void stick_direction_changed(int stick, int direction, int gamepad_index);

public slots:
  void update_gamepads() {
    GamepadUpdate();
    for (int i = 0; i != GAMEPAD_COUNT; i++) {
      update_gamepad(i);
    }
  }

private:
  void update_gamepad(int gamepad_index) {
    auto gamepad = _gamepads[gamepad_index];
    auto device = static_cast<GAMEPAD_DEVICE>(gamepad_index);

    // check for a change in connected status
    if (!gamepad->is_connected() && GamepadIsConnected(device)) {
      gamepad->mark_connected();
      emit gamepad->connected();
      emit gamepad_connected(gamepad_index);
    }
    if (gamepad->is_connected() && !GamepadIsConnected(device)) {
      gamepad->mark_disconnected();
      emit gamepad->disconnected();
      emit gamepad_disconnected(gamepad_index);
    }

    if (GamepadIsConnected(device)) {
      // check for a change in button presses
      for (int button = 0; button != BUTTON_COUNT; button++) {
        const auto cast_button = static_cast<GAMEPAD_BUTTON>(button);
        if (GamepadButtonTriggered(device, cast_button)) {
          emit gamepad->button_pressed(button);
          emit button_pressed(button, device);
        } else if (GamepadButtonReleased(device, cast_button)) {
          emit gamepad->button_released(button);
          emit button_released(button, device);
        }
      }

      // check for a change in triggers
      for (int trigger = 0; trigger != TRIGGER_COUNT; trigger++) {
        const auto cast_trigger = static_cast<GAMEPAD_TRIGGER>(trigger);
        if (GamepadTriggerTriggered(device, cast_trigger)) {
          emit gamepad->trigger_pressed(trigger);
          emit trigger_pressed(trigger, device);
        } else if (GamepadTriggerReleased(device, cast_trigger)) {
          emit gamepad->trigger_released(trigger);
          emit trigger_released(trigger, device);
        }
      }

      // check for a change in stick direction
      for (int stick = 0; stick != STICK_COUNT; stick++) {
        const auto cast_stick = static_cast<GAMEPAD_STICK>(stick);
        for (int direction = 0; direction != STICKDIR_COUNT; direction++) {
          const auto cast_direction = static_cast<GAMEPAD_STICKDIR>(direction);
          if (GamepadStickDirTriggered(device, cast_stick, cast_direction)) {
            emit gamepad->stick_direction_changed(stick, direction);
            emit stick_direction_changed(stick, direction, device);
          }
        }
      }

      // update trigger depth and joystick position
      gamepad->update_trigger_value(TRIGGER_LEFT,
          GamepadTriggerValue(device, TRIGGER_LEFT));
      gamepad->update_trigger_value(TRIGGER_RIGHT,
          GamepadTriggerValue(device, TRIGGER_RIGHT));

      int x, y;
      GamepadStickXY(device, STICK_LEFT, &x, &y);
      gamepad->update_joystick_value(STICK_LEFT, x, y);
      GamepadStickXY(device, STICK_RIGHT, &x, &y);
      gamepad->update_joystick_value(STICK_RIGHT, x, y);
    }
  }

private:
  std::vector<Gamepad*> _gamepads;
  QTimer *_timer;
};

#endif // GAMEPAD_CLASS_H
