#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#include <QByteArray>
#include <QObject>
#include <QSerialPort>

#include <cstdint>  // for uint64_t

/** Serial Communicator
 *
 * Communicates over a given serial port using delimiters that mark the
 * beginning and end of messages.  Has a slot for a message to be sent, and
 * emits a signal when a message (or command) is received between the
 * delimiters.  This class does not know how to interpret the sent or received
 * messages or commands, just relays them - like a mail main.
 *
 * Default delimiters are '<' for beginning and '>' for end.
 *
 * For example:
 *
 *   QSerialPort serialPort;
 *   SerialCommunicator communicator(&serialPort);
 *   communicator.set_start('[');
 *   communicator.set_end(']');
 *   connect(&communicator, &SerialCommunicator::received,
 *           [](const QString &msg) { qDebug() << "raw message: " << msg; });
 *   connect(this, &ThisClass::to_send,
 *           &communicator, &SerialCommunicator::send);
 */
class SerialCommunicator : public QObject {
  Q_OBJECT

public:
  SerialCommunicator(QSerialPort *port, QObject *parent = nullptr);

  QSerialPort* port() { return _port; }
  const QSerialPort* port() const { return _port; }

  uint64_t num_received() const { return _num_received; }
  uint64_t num_sent()     const { return _num_sent;     }

  char start() const { return _start; }
  char end()   const { return _end;   }
  void set_start(char start) { _start = start; }
  void set_end  (char end  ) { _end   = end;   }

public slots:
  void send(const QString &msg) const;

signals:
  void received(const QString &msg) const;

private slots:
  void read();

private:
  QSerialPort *_port;
  mutable uint64_t _num_received;
  mutable uint64_t _num_sent;
  char _start;
  char _end;
  QByteArray _data;
};

#endif // SERIAL_COMMUNICATOR_H
