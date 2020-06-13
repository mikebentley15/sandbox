#include "Gamepad.h"

#include <QtCore/QCoreApplication>

#include <iostream>

class GamepadLogger {
public:
  GamepadLogger(GamepadManager *manager) {
    //connect_manager(manager);
    for (size_t i = 0; i < manager->size(); i++) {
      connect_gamepad(manager->at(i));
    }
  }

private:
  auto mklogfn_0(const std::string name) const {
    auto log_fn = [name]() {
      std::cout << name << "()\n";
    };
    return log_fn;
  }

  auto mklogfn_1(const std::string name) const {
    auto log_fn = [name](int a) {
      std::cout << name << "(" << a << ")\n";
    };
    return log_fn;
  }

  auto mklogfn_2(const std::string name) const {
    auto log_fn = [name](int a, int b) {
      std::cout << name << "(" << a << ", " << b << ")\n";
    };
    return log_fn;
  }

  auto mklogfn_3(const std::string name) const {
    auto log_fn = [name](int a, int b, int c) {
      std::cout << name << "(" << a << ", " << b << ", " << c << ")\n";
    };
    return log_fn;
  }

  auto mklogfn_1d(const std::string name) const {
    auto log_fn = [name](double a) {
      std::cout << name << "(" << a << ")\n";
    };
    return log_fn;
  }

  auto mklogfn_2d(const std::string name) const {
    auto log_fn = [name](double a, double b) {
      std::cout << name << "(" << a << ", " << b << ")\n";
    };
    return log_fn;
  }

  void connect_manager(GamepadManager *manager) {
    QObject::connect(manager, &GamepadManager::gamepad_connected,
                     mklogfn_1("manager.gamepad_connected"));
    QObject::connect(manager, &GamepadManager::gamepad_disconnected,
                     mklogfn_1("manager.gamepad_disconnected"));
    QObject::connect(manager, &GamepadManager::button_pressed,
                     mklogfn_2("manager.button_pressed"));
    QObject::connect(manager, &GamepadManager::button_released,
                     mklogfn_2("manager.button_released"));
    QObject::connect(manager, &GamepadManager::trigger_pressed,
                     mklogfn_2("manager.trigger_pressed"));
    QObject::connect(manager, &GamepadManager::trigger_released,
                     mklogfn_2("manager.trigger_released"));
    QObject::connect(manager, &GamepadManager::stick_direction_changed,
                     mklogfn_3("manager.stick_direction_changed"));
  }

  void connect_gamepad(Gamepad *gamepad) {
    std::string name = "gamepad_" + std::to_string(gamepad->id());
    QObject::connect(gamepad, &Gamepad::connected,
                     mklogfn_0(name + ".connected"));
    QObject::connect(gamepad, &Gamepad::disconnected,
                     mklogfn_0(name + ".disconnected"));
    //QObject::connect(gamepad, &Gamepad::button_pressed,
    //                 mklogfn_1(name + ".button_pressed"));
    //QObject::connect(gamepad, &Gamepad::button_released,
    //                 mklogfn_1(name + ".button_released"));
    //QObject::connect(gamepad, &Gamepad::trigger_pressed,
    //                 mklogfn_1(name + ".trigger_pressed"));
    //QObject::connect(gamepad, &Gamepad::trigger_released,
    //                 mklogfn_1(name + ".trigger_released"));
    //QObject::connect(gamepad, &Gamepad::stick_direction_changed,
    //                 mklogfn_2(name + ".stick_direction_changed"));
    QObject::connect(gamepad, &Gamepad::dpad_up_pressed,
                     mklogfn_0(name + ".dpad_up_pressed"));
    QObject::connect(gamepad, &Gamepad::dpad_up_released,
                     mklogfn_0(name + ".dpad_up_released"));
    QObject::connect(gamepad, &Gamepad::dpad_down_pressed,
                     mklogfn_0(name + ".dpad_down_pressed"));
    QObject::connect(gamepad, &Gamepad::dpad_down_released,
                     mklogfn_0(name + ".dpad_down_released"));
    QObject::connect(gamepad, &Gamepad::dpad_left_pressed,
                     mklogfn_0(name + ".dpad_left_pressed"));
    QObject::connect(gamepad, &Gamepad::dpad_left_released,
                     mklogfn_0(name + ".dpad_left_released"));
    QObject::connect(gamepad, &Gamepad::dpad_right_pressed,
                     mklogfn_0(name + ".dpad_right_pressed"));
    QObject::connect(gamepad, &Gamepad::dpad_right_released,
                     mklogfn_0(name + ".dpad_right_released"));
    QObject::connect(gamepad, &Gamepad::start_pressed,
                     mklogfn_0(name + ".start_pressed"));
    QObject::connect(gamepad, &Gamepad::start_released,
                     mklogfn_0(name + ".start_released"));
    QObject::connect(gamepad, &Gamepad::select_pressed,
                     mklogfn_0(name + ".select_pressed"));
    QObject::connect(gamepad, &Gamepad::select_released,
                     mklogfn_0(name + ".select_released"));
    QObject::connect(gamepad, &Gamepad::L1_pressed,
                     mklogfn_0(name + ".L1_pressed"));
    QObject::connect(gamepad, &Gamepad::L1_released,
                     mklogfn_0(name + ".L1_released"));
    QObject::connect(gamepad, &Gamepad::R1_pressed,
                     mklogfn_0(name + ".R1_pressed"));
    QObject::connect(gamepad, &Gamepad::R1_released,
                     mklogfn_0(name + ".R1_released"));
    QObject::connect(gamepad, &Gamepad::L2_pressed,
                     mklogfn_0(name + ".L2_pressed"));
    QObject::connect(gamepad, &Gamepad::L2_released,
                     mklogfn_0(name + ".L2_released"));
    QObject::connect(gamepad, &Gamepad::R2_pressed,
                     mklogfn_0(name + ".R2_pressed"));
    QObject::connect(gamepad, &Gamepad::R2_released,
                     mklogfn_0(name + ".R2_released"));
    QObject::connect(gamepad, &Gamepad::L3_pressed,
                     mklogfn_0(name + ".L3_pressed"));
    QObject::connect(gamepad, &Gamepad::L3_released,
                     mklogfn_0(name + ".L3_released"));
    QObject::connect(gamepad, &Gamepad::R3_pressed,
                     mklogfn_0(name + ".R3_pressed"));
    QObject::connect(gamepad, &Gamepad::R3_released,
                     mklogfn_0(name + ".R3_released"));
    QObject::connect(gamepad, &Gamepad::X_pressed,
                     mklogfn_0(name + ".X_pressed"));
    QObject::connect(gamepad, &Gamepad::X_released,
                     mklogfn_0(name + ".X_released"));
    QObject::connect(gamepad, &Gamepad::Y_pressed,
                     mklogfn_0(name + ".Y_pressed"));
    QObject::connect(gamepad, &Gamepad::Y_released,
                     mklogfn_0(name + ".Y_released"));
    QObject::connect(gamepad, &Gamepad::A_pressed,
                     mklogfn_0(name + ".A_pressed"));
    QObject::connect(gamepad, &Gamepad::A_released,
                     mklogfn_0(name + ".A_released"));
    QObject::connect(gamepad, &Gamepad::B_pressed,
                     mklogfn_0(name + ".B_pressed"));
    QObject::connect(gamepad, &Gamepad::B_released,
                     mklogfn_0(name + ".B_released"));
    QObject::connect(gamepad, &Gamepad::L2_value_changed,
                    mklogfn_1d(name + ".L2_value_changed"));
    QObject::connect(gamepad, &Gamepad::R2_value_changed,
                    mklogfn_1d(name + ".R2_value_changed"));
    QObject::connect(gamepad, &Gamepad::left_joystick_changed,
                    mklogfn_2d(name + ".left_joystick_changed"));
    QObject::connect(gamepad, &Gamepad::right_joystick_changed,
                    mklogfn_2d(name + ".right_joystick_changed"));
  }
};

int main(int argCount, char* argList[]) {
  QCoreApplication app(argCount, argList);
  GamepadManager manager(&app);
  GamepadLogger logger(&manager);
  std::cout << "Press CTRL-C to quit\n\n";
  return app.exec();
}
