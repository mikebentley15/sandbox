#include "SerialCommunicator.h"

#include <QtGlobal>
#include <QtDebug>
#include <QIODevice>

#include <stdexcept>


/** [Public Methods] **/

SerialCommunicator::SerialCommunicator(QSerialPort *port, QObject *parent)
  : QObject(parent)
  , _port(port)
  , _num_received(0)
  , _num_sent(0)
  , _start('<')
  , _end('>')
  , _data()
{
  if (port == nullptr) {
    throw std::invalid_argument("You must provide a non-null serial port object");
  }

  // check that port is initialized and open
  if (!port->isOpen() || !port->isReadable() || ! port->isWritable()) {
    throw std::invalid_argument("Serial port must be open with read+write");
  }

  if (port->error() != QSerialPort::NoError) {
    throw std::invalid_argument("Serial port has an error: code "
                                + std::to_string(int(port->error())));
  }

  // connect up the port to receive messages from it
  connect(port, &QIODevice::readyRead,
          this, &SerialCommunicator::read);
}


/** [Public Slots] **/

void SerialCommunicator::send(const QString &msg) const {
  _num_sent++;
  qWarning() << "SerialCommunicator::send() is unimplemented";
  QByteArray to_write = _start + msg.toUtf8() + _end;

  // send
  auto bytes_written = _port->write(to_write);

  // make sure it sent
  if (bytes_written == -1) {
    qFatal("Failed to send message on serial port");
  }
  if (bytes_written != to_write.size()) {
    qFatal("Failed to send entire message");
  }
}


/** [Private Slots] **/

void SerialCommunicator::read() {
  // read from the port
  _data.append(_port->readAll());
  if (_port->error() == QSerialPort::ReadError) {
    qFatal("Failed to read from port %s, error: %s",
           _port->portName().data(), _port->errorString().toStdString());
  }

  // for each beginning and ending, emit received with interior
  int idx = 0;
  while ((idx = _data.indexOf(_start, idx)) != -1) {
    int end_idx = _data.indexOf(_end, idx + 1);

    // missing end character - nothing more to parse for now
    if (end_idx == -1) {
      if (idx > 0) {
        // remove everything before the start character
        qDebug() << "_data before left stripping:      " << _data;
        _data = _data.right(_data.size() - idx);
        qDebug() << "_data after  left stripping:      " << _data;
      }
      break; // done parsing for now (missing end character)
    }

    // has the end character.  extract the command and emit
    _num_received++;
    auto command = QString::fromUtf8(_data.data() + idx + 1, end_idx - idx);
    qDebug() << "_data before removing one message:" << _data;
    _data = _data.right(_data.size() - end_idx - 1);
    qDebug() << "_data after  removing one message:" << _data;
    emit received(command);
  }
}
