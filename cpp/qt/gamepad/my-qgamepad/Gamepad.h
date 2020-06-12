#ifndef GAMEPAD_CLASS_H
#define GAMEPAD_CLASS_H

#include "gamepad/gamepad.h"

#include <QtCore/QObject>
#include <QtCore/QTimer>

#include <vector>

class GamepadManager;

class Gamepad : public QObject {
  Q_OBJECT

public:
  int id() const { return _id; };
  bool is_connected() const { return _is_connected; }

  // TODO: add getters for each button state

signals:
  void connected();
  void disconnected();
  void button_pressed(int button);
  void button_released(int button);
  void trigger_pressed(int trigger);
  void trigger_released(int trigger);
  void stick_direction_changed(int stick, int direction);

  // TODO: add signals like button_a() and so forth

private:
  // Private constructor that only GamepadManager can call
  Gamepad(QObject *parent, int id)
    : QObject(parent)
    , _id(id)
    , _is_connected(false)
  {}

  void mark_connected()    { _is_connected = true; }
  void mark_disconnected() { _is_connected = false; }

private slots:
  // TODO: emit signals for things like button_a() and so forth
  // TODO: implement a mapping for button integer to button type
  // TODO: allow the user to modify the mapping

private:
  const int _id;       // gamepad index
  bool _is_connected;  // cached connection result to allow event detection

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

  //Q_PROPERTY(int deviceId READ deviceId WRITE setDeviceId NOTIFY deviceIdChanged)
  //Q_PROPERTY(bool connected READ isConnected NOTIFY connectedChanged)
  //Q_PROPERTY(QString name READ name NOTIFY nameChanged)
  //Q_PROPERTY(double axisLeftX READ axisLeftX NOTIFY axisLeftXChanged)
  //Q_PROPERTY(double axisLeftY READ axisLeftY NOTIFY axisLeftYChanged)
  //Q_PROPERTY(double axisRightX READ axisRightX NOTIFY axisRightXChanged)
  //Q_PROPERTY(double axisRightY READ axisRightY NOTIFY axisRightYChanged)
  //Q_PROPERTY(bool buttonA READ buttonA NOTIFY buttonAChanged)
  //Q_PROPERTY(bool buttonB READ buttonB NOTIFY buttonBChanged)
  //Q_PROPERTY(bool buttonX READ buttonX NOTIFY buttonXChanged)
  //Q_PROPERTY(bool buttonY READ buttonY NOTIFY buttonYChanged)
  //Q_PROPERTY(bool buttonL1 READ buttonL1 NOTIFY buttonL1Changed)
  //Q_PROPERTY(bool buttonR1 READ buttonR1 NOTIFY buttonR1Changed)
  //Q_PROPERTY(double buttonL2 READ buttonL2 NOTIFY buttonL2Changed)
  //Q_PROPERTY(double buttonR2 READ buttonR2 NOTIFY buttonR2Changed)
  //Q_PROPERTY(bool buttonSelect READ buttonSelect NOTIFY buttonSelectChanged)
  //Q_PROPERTY(bool buttonStart READ buttonStart NOTIFY buttonStartChanged)
  //Q_PROPERTY(bool buttonL3 READ buttonL3 NOTIFY buttonL3Changed)
  //Q_PROPERTY(bool buttonR3 READ buttonR3 NOTIFY buttonR3Changed)
  //Q_PROPERTY(bool buttonUp READ buttonUp NOTIFY buttonUpChanged)
  //Q_PROPERTY(bool buttonDown READ buttonDown NOTIFY buttonDownChanged)
  //Q_PROPERTY(bool buttonLeft READ buttonLeft NOTIFY buttonLeftChanged)
  //Q_PROPERTY(bool buttonRight READ buttonRight NOTIFY buttonRightChanged)
  //Q_PROPERTY(bool buttonCenter READ buttonCenter NOTIFY buttonCenterChanged)
  //Q_PROPERTY(bool buttonGuide READ buttonGuide NOTIFY buttonGuideChanged)

  Q_PROPERTY(unsigned poll_interval READ poll_interval WRITE poll_interval)

public:
//  static const char* button_names[] = {
//    "d-pad up",
//    "d-pad down",
//    "d-pad left",
//    "d-pad right",
//    "start",
//    "back",
//    "left thumb",
//    "right thumb",
//    "left shoulder",
//    "right shoulder",
//    "???",
//    "???",
//    "A",
//    "B",
//    "X",
//    "Y"
//  };

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
  Gamepad* operator[](size_t i)             { return _gamepads[i];    }
  Gamepad* at(size_t i)                     { return _gamepads.at(i); }
  const Gamepad* operator[](size_t i) const { return _gamepads[i];    }
  const Gamepad* at(size_t i)         const { return _gamepads.at(i); }

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

      // TODO: check for a change in stick position and length
    }
  }

private:
  std::vector<Gamepad*> _gamepads;
  QTimer *_timer;
};

#endif // GAMEPAD_CLASS_H
