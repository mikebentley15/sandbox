#include "Controller.h"
#include "FakeReader.h"
#include "LocalizingFilter.h"
#include "Logger.h"
#include "Registrar.h"

#include <QCoreApplication>
#include <QTime>
#include <QByteArray>
#include <QTimer>
#include <QObject>

#include <iostream>

void timestampMessageHandler(QtMsgType type, const QMessageLogContext &context,
                             const QString &msg)
{
  QByteArray localMsg = msg.toLocal8Bit();
  std::cerr << QTime::currentTime().toString().toStdString() << ": "
            << msg.toStdString() << std::endl;
}

int main(int argCount, char *argList[]) {
  qInstallMessageHandler(timestampMessageHandler);
  QCoreApplication app(argCount, argList);

  Controller       controller;
  FakeReader       reader;
  LocalizingFilter filter;
  Registrar        registrar;
  Logger           logger;

  // log everything
  QObject::connect(&app, &QCoreApplication::aboutToQuit,
                   &logger, &Logger::log_quit);
  QObject::connect(&controller, &Controller::robot_command,
                   &logger, &Logger::log_robot_pose);
  QObject::connect(&reader, &FakeReader::new_sensor_reading,
                   &logger, &Logger::log_sensor_reading);
  QObject::connect(&filter, &LocalizingFilter::updated_belief,
                   &logger, &Logger::log_belief);
  QObject::connect(&registrar, &Registrar::new_registered_pose,
                   &logger, &Logger::log_registered_pose);

  // 1 second of running
  QTimer::singleShot(1000, &app, &QCoreApplication::quit);
  return app.exec();
}
