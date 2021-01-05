#include <QCoreApplication>
#include <QSerialPort>
#include <QStringList>
#include <QTextStream>

#include <iostream>

int main(int argc, char *argv[])
{
  std::cout << "Creating QCoreApplication" << std::endl;
  QCoreApplication coreApplication(argc, argv);
  std::cout << "  done" << std::endl;
  const int argumentCount = QCoreApplication::arguments().size();
  const QStringList argumentList = QCoreApplication::arguments();

  //QTextStream out(stdout);
  //out << "hi there from the QTextStream\n";

//    if (argumentCount == 1) {
//        out << QObject::tr("Usage: %1 <serialportname> [baudrate]")
//                          .arg(argumentList.first())
//                       << "\n";
//        return 1;
//    }

  const QString serialPortName = "/dev/ttyACM0"; //"COM5"; //argumentList.at(1);
  std::cout << "About to open port: " << serialPortName.toStdString() << std::endl;
  QSerialPort serialPort;
  serialPort.setPortName(serialPortName);
  std::cout << "Opened port:        " << serialPortName.toStdString() << std::endl;

  const int serialPortBaudRate = (argumentCount > 2)
          ? argumentList.at(2).toInt() : QSerialPort::Baud9600;
  serialPort.setBaudRate(serialPortBaudRate);
  std::cout << "Set baud rate to:   " << serialPortBaudRate << std::endl;

  if (!serialPort.open(QIODevice::ReadOnly)) {
    std::cout << "Failed to open port " << serialPortName.toStdString()
              << ", error: " << serialPort.errorString().toStdString()
              << std::endl;
    return 1;
  }

  QByteArray readData = serialPort.readAll();
  while (serialPort.waitForReadyRead(5000)){

    std::cout << "\n" << readData.toStdString() << std::endl;
    readData.append(serialPort.readAll());
  }

  if (serialPort.error() == QSerialPort::ReadError) {
    std::cout << "Failed to read from port " << serialPortName.toStdString()
              << ", error: " << serialPort.errorString().toStdString()
              << std::endl;
    return 1;
  } else if (serialPort.error() == QSerialPort::TimeoutError && readData.isEmpty()) {
    std::cout << "No data was currently available for reading from port "
              << serialPortName.toStdString()
              << std::endl;
    return 0;
  }
  std::cout << "Data successfully received from port "
            << serialPortName.toStdString()
            << std::endl;
  std::cout << "\n" << readData.toStdString() << std::endl;

  return 0;
}
