#include "SerialCommunicator.h"
#include "MessageParser.h"
#include "MessageGenerator.h"

#include <QCoreApplication>
#include <QSerialPort>
#include <QString>
#include <QTimer>

#include <iostream>

int main (int argc, char *argv[]) {
  QCoreApplication app(argc, argv);
  const QStringList argList = QCoreApplication::arguments();

  // print help (if --help was given)
  if (argList.size() > 1 && argList.at(1) == "--help") {
    std::cout
      << "Usage:\n"
         "  " << argList.at(0).toStdString() << " [<device>] [<baud>]\n"
         "\n"
         "Description:\n"
         "  Repeatedly sends '<msg hello world>' over the serial port given\n"
         "  by <device>.  It will also listen to messages received from the\n"
         "  serial port of the format '<msg [message]>' and will print the\n"
         "  messages received.\n"
         "\n"
         "Positional Arguments:\n"
         "  <device>   By default, it uses /dev/ttyACM0.  The serial port\n"
         "             device.\n"
         "  <baud>     Baud rate.  By default, uses 9600.\n"
      << std::endl;
    return 0;
  }

  // argument parsing
  QString port("/dev/ttyACM0");
  QSerialPort::BaudRate baud(QSerialPort::Baud9600);
  if (argList.size() > 1) {
    port = argList.at(1);
  }
  if (argList.size() > 2) {
    bool ok = true;
    baud = QSerialPort::BaudRate(argList.at(2).toInt(&ok));
    if (!ok) {
      std::cerr << "Could not convert '" << argList.at(2).toStdString()
                << "' to an integer" << std::endl;
      return 1;
    }
  }

  // print settings
  std::cout
    << argList.at(0).toStdString() << "\n"
    << "device:  " << port.toStdString() << "\n"
    << "baud:    " << int(baud)          << "\n"
    << std::endl;

  QSerialPort serialPort;
  int return_code = -1;
  try {
    // open the serial port connection
    serialPort.setPortName(port);
    serialPort.setBaudRate(baud);
    if (!serialPort.open(QIODevice::ReadWrite)) {
      std::cout << "Failed to open port " << port.toStdString()
                << ", error: " << serialPort.errorString().toStdString()
                << std::endl;
      return 1;
    }

    // create other filters
    SerialCommunicator communicator(&serialPort);
    MessageGenerator generator;
    MessageParser    parser;
    QTimer           heartbeat_timer;

    communicator.set_start('<');
    communicator.set_end('>');
    heartbeat_timer.start(1000); // heartbeat every 1 second

    //
    // make connections
    //

    // cleanup
    QObject::connect(&app, &QCoreApplication::aboutToQuit,
        [&serialPort]() { serialPort.close(); });
    QObject::connect(&serialPort, &QSerialPort::aboutToClose,
        []() { std::cout << "about to close serial port" << std::endl; });

    // received message pipeline
    QObject::connect(&communicator, &SerialCommunicator::received,
                     &parser,       &MessageParser::parse);
    QObject::connect(&parser,       &MessageParser::new_message,
        [](const QString &msg) {
          std::cout << "received:  '" << msg.toStdString() << "'" << std::endl;
        });

    // message generation pipeline
    QObject::connect(&heartbeat_timer, &QTimer::timeout,
        [&generator]() {
          static int i = 1;
          generator.message("hello world - " + QString::number(i++));
        });
    QObject::connect(&generator,    &MessageGenerator::to_send,
                     &communicator, &SerialCommunicator::send);

    // start the event loop
    return_code = app.exec();
  } catch (...) {
    serialPort.close();
    return_code = -1;
    throw; // rethrow
  }

  return return_code;
}
