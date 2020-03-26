#ifndef SENDER_H
#define SENDER_H

#include "MessageTypes.h"

#include <QObject>

class Sender : public QObject {
  Q_OBJECT

public:
  Sender(QObject *parent = nullptr) : QObject(parent) {}

signals:
  void send_1(int a, int b, int c, int d, int e);
  void send_2(const Derived &d);
};

#endif // SENDER_H
