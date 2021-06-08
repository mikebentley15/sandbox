#ifndef MESSAGE_PARSER_H
#define MESSAGE_PARSER_H

#include <QObject>

/** Message Parser
 *
 * Parses messages from the serial port protocol into data structures useful by
 * the application.
 *
 * This class is completely stateless.  It follows a filter paradigm using Qt's
 * signal and slot mechanisms.
 *
 * For example:
 *
 *   MessageParser* parser = new MessageParser(this);
 *   connect(this, &ThisClass::received_message,
 *           parser, &MessageParser::parse);
 *   connect(parser, &MessageParser::new_message,
 *           [](const QString &msg) { qDebug() << "Message: " << msg; });
 */
class MessageParser : public QObject {
  Q_OBJECT

public:
  MessageParser(QObject *parent = nullptr);

public slots:
  void parse(const QString &to_parse) const;

signals:
  // parsed messages
  // TODO: implement more than just the 'msg <message>' command
  void new_message(const QString &msg) const;

};

#endif // MESSAGE_PARSER_H
