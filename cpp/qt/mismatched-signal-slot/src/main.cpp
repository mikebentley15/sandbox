#include "Receiver.h"
#include "Sender.h"
#include "MessageTypes.h"

#include <QCoreApplication>
#include <QTimer>
#include <QtDebug>

int main(int argCount, char* argList[]) {
  QCoreApplication app(argCount, argList);

  Sender s(&app);
  Receiver r(&app);

  // Note: order of connections matter.  It is first in first out, meaning the
  //   connections are called in the order of connection for a particular
  //   emitted signal.

  // log emitted signals from here
  QObject::connect(&s, &Sender::send_1,
    [](int a, int b, int c, int d, int e) -> void {
      qDebug() << "send_1   ("
               << a << "," << b << "," << c << "," << d << "," << e
               << ")";
    });
  QObject::connect(&s, &Sender::send_2,
    [](const Derived &d) -> void {
      d.print_me("send_2:");
    });

  // connect the sender to the receiver
  QObject::connect(&s, &Sender::send_1, &r, &Receiver::receive_1);

  // send messages from here
  QTimer t1(&app);
  t1.setInterval(99);
  QObject::connect(&t1, &QTimer::timeout,
    [&s]() -> void {
      static int a = 4, b = 3, c = 2, d = 1, e = 0;
      emit s.send_1(++a, ++b, ++c, ++d, ++e);
    });
  t1.start();

  QTimer t2(&app);
  t2.setInterval(150);
  QObject::connect(&t2, &QTimer::timeout,
    [&s]() -> void {
      static Derived d{};
      d.age += 1;
      emit s.send_2(d);
    });

  // Stop timer 1 after 500 milliseconds and then start timer 2
  QTimer::singleShot(500, &t1, &QTimer::stop);
  QTimer::singleShot(500, &t2, &QTimer::start);

  // run for 1 second
  QTimer::singleShot(1000, &app, &QCoreApplication::quit);
  return app.exec();
}
