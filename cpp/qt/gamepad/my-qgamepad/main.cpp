#include "Gamepad.h"

#include <QtCore/QCoreApplication>

#include <iostream>

class GamepadLogger {
public:
  GamepadLogger(GamepadManager *manager) {
    QObject::connect(manager, &GamepadManager::gamepad_connected,
      [](int index) {
        std::cout << "manager.gamepad_connected(" << index << ")\n";
      });
    QObject::connect(manager, &GamepadManager::gamepad_disconnected,
      [](int index) {
        std::cout << "manager.gamepad_disconnected(" << index << ")\n";
      });
    QObject::connect(manager, &GamepadManager::button_pressed,
      [](int button, int index) {
        std::cout << "manager.button_pressed(" << button << ", " << index
                  << ")\n";
      });
    QObject::connect(manager, &GamepadManager::button_released,
      [](int button, int index) {
        std::cout << "manager.button_released(" << button << ", " << index
                  << ")\n";
      });
    QObject::connect(manager, &GamepadManager::trigger_pressed,
      [](int trigger, int index) {
        std::cout << "manager.trigger_pressed(" << trigger << ", " << index
                  << ")\n";
      });
    QObject::connect(manager, &GamepadManager::trigger_released,
      [](int trigger, int index) {
        std::cout << "manager.trigger_released(" << trigger << ", " << index
                  << ")\n";
      });
    QObject::connect(manager, &GamepadManager::stick_direction_changed,
      [](int stick, int direction, int index) {
        std::cout << "manager.stick_direction_changed("
                  << stick << ", "
                  << direction << ", "
                  << index << ")\n";
      });
    // TODO: log each of the gamepad signals too
  }
};

int main(int argCount, char* argList[]) {
  QCoreApplication app(argCount, argList);
  GamepadManager manager(&app);
  GamepadLogger logger(&manager);
  std::cout << "Press CTRL-C to quit\n\n";
  return app.exec();
}
