#include "MessageParser.h"

#include <QtGlobal> // for qFatal()

/** [Public Methods] **/

MessageParser::MessageParser(QObject *parent) : QObject(parent) {}


/** [Public Slots] **/

void MessageParser::parse(const QString &to_parse) const {
  if (to_parse.startsWith("msg ")) {
    QString msg = to_parse.right(to_parse.size() - 4);
    emit new_message(msg);
  // } else if (to_parse.startsWith("move ")) {
  } else {
    qFatal("MessageParser: parsing error: unrecognized command type: '%s'",
           to_parse.data());
  }
}
