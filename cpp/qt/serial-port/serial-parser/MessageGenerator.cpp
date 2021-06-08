#include "MessageGenerator.h"

/** [Public Methods] **/

MessageGenerator::MessageGenerator(QObject *parent) : QObject(parent) {}


/** [Public Slots] **/

void MessageGenerator::message(const QString &msg) const {
  emit to_send("msg " + msg);
}
