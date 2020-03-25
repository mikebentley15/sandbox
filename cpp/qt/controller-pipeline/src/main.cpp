#include "Controller.h"
#include "FakeReader.h"
#include "LocalizingFilter.h"
#include "Logger.h"
#include "Registrar.h"

#include <QCoreApplication>
#include <QTimer>
#include <QObject>

int main(int argCount, char *argList[]) {
  QCoreApplication app(argCount, argList);

  Logger logger;
  QObject::connect(&app, &QCoreApplication::aboutToQuit,
                   &logger, &Logger::log_quit);

  // 1 second of running
  QTimer::singleShot(1000, &app, &QCoreApplication::quit);
  return app.exec();
}
