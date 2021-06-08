#ifndef MESSAGE_GENERATOR_H
#define MESSAGE_GENERATOR_H

#include <QObject>

/** Message Generator
 *
 * Converts different types of messages into the string representation using
 * the established serial port communication protocol.
 *
 * This class is completely stateless.  It follows a filter paradigm using Qt's
 * signal and slot mechanisms.
 *
 * For example:
 *
 *   MessageGenerator* generator = new MessageGenerator(parent);
 *   connect(this, &ThisClass::new_message,
 *           generator, &MessageGenerator::message);
 *   connect(generator, &MessageGenerator::to_send,
 *           [](const QString &msg) { qDebug() << msg; });
 */
class MessageGenerator : public QObject {
  Q_OBJECT

public:
  MessageGenerator(QObject *parent = nullptr);

public slots:
  void message(const QString &msg) const;

signals:
  void to_send(const QString &msg) const;
};

#endif // MESSAGE_GENERATOR_H
