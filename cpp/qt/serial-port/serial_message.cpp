#include <QCoreApplication>
#include <QSerialPort>
#include <QString>

#include <iostream>

int main (int arg_count, char *arg_list[]) {
  QCoreApplication app(arg_count, arg_list);

  QString port("/dev/ttyACM0");
  QSerialPort serial(port);
  serial.setBaudRate(QSerialPort::Baud9600);
  if (!serial.open(QIODevice::WriteOnly)) {
    std::cerr << "Could not open " << port.toStdString() << std::endl;
    return 1;
  }

  std::string msg = "message\n";
  auto bytes_written = serial.write(msg.c_str());
  if (bytes_written == -1) {
    std::cerr << "Failed to send message on serial port" << std::endl;
    return 1;
  }
  if (bytes_written != msg.size()) {
    std::cerr << "Failed to send entire message" << std::endl;
    return 1;
  }
  if (!serial.waitForBytesWritten(5000)) {
    std::cerr << "Timeout for writing to serial port" << std::endl;
    return 1;
  }

  std::cout << "Message successfully sent!" << std::endl;
  
  return 0;
}
