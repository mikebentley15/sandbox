#ifndef RECEIVER_H
#define RECEIVER_H

#include "MessageTypes.h"

#include <QObject>

#include <QtDebug>

class Receiver : public QObject {
  Q_OBJECT

public:
  Receiver(QObject *parent = nullptr) : QObject(parent) {}

public slots:
  void receive_1(int a, int b) {
    qDebug() << "receive_1(" << a << "," << b << ")";
  }

  void receive_2(const Base &b) {
    b.print_me("receive_2:");
  }
};

#endif // RECEIVER_H
