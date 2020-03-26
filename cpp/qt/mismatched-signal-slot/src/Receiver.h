#ifndef RECEIVER_H
#define RECEIVER_H

#include "MessageTypes.h"

#include <QObject>
#include <QtDebug>

#include <string>

class Receiver : public QObject {
  Q_OBJECT

public:
  Receiver(QObject *parent = nullptr) : QObject(parent) {}

public slots:
  void receive_1(int a, int b) {
    qDebug() << current_thread_id() << ":"
             << "receive_1(" << a << "," << b << ")";
  }

  void receive_2(const Base &b) {
    b.print_me(std::string(current_thread_id()) + " : receive_2:");
  }
};

#endif // RECEIVER_H
