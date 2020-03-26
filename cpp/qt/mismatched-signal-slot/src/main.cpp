#include "Receiver.h"
#include "Sender.h"
#include "MessageTypes.h"

#include <QCoreApplication>
#include <QTimer>
#include <QtDebug>
#include <QThread>

int main(int argCount, char* argList[]) {
  QCoreApplication app(argCount, argList);
  qRegisterMetaType<Base>("Base");
  qRegisterMetaType<Derived>("Derived");

  // Put sender and receiver in different threads
  QThread s_thread;
  QThread r_thread;
  s_thread.setObjectName(" sending ");
  r_thread.setObjectName("receiving");
  QObject::connect(&app, &QCoreApplication::aboutToQuit,
                   &s_thread, &QThread::quit);
  QObject::connect(&app, &QCoreApplication::aboutToQuit,
                   &r_thread, &QThread::quit);

  Sender s;
  Receiver r;
  s.moveToThread(&s_thread);
  r.moveToThread(&r_thread);

  // Note: order of connections matter.  It is first in first out, meaning the
  //   connections are called in the order of connection for a particular
  //   emitted signal.

  // log emitted signals from here
  QObject::connect(&s, &Sender::send_1,
    [](int a, int b, int c, int d, int e) -> void {
      qDebug() << current_thread_id() << ":"
               << "send_1     ("
               << a << "," << b << "," << c << "," << d << "," << e
               << ")";
    });
  QObject::connect(&s, &Sender::send_2,
    [](const Derived &d) -> void {
      d.print_me(std::string(current_thread_id()) + " : send_2:   ");
    });

  // connect the sender to the receiver
  // Note: I believe Qt::QueuedConnection is the default
  QObject::connect(&s, &Sender::send_1, &r, &Receiver::receive_1, Qt::QueuedConnection);
  QObject::connect(&s, &Sender::send_2, &r, &Receiver::receive_2, Qt::QueuedConnection);

  // send messages from here
  QTimer t1;
  t1.moveToThread(&s_thread);
  t1.setInterval(99);
  QObject::connect(&s_thread, &QThread::finished, &t1, &QTimer::stop);
  QObject::connect(&t1, &QTimer::timeout,
    [&s]() -> void {
      static int a = 4, b = 3, c = 2, d = 1, e = 0;
      emit s.send_1(++a, ++b, ++c, ++d, ++e);
    });
  QObject::connect(&s_thread, &QThread::started, &t1, qOverload<>(&QTimer::start));

  QTimer t2;
  t2.setInterval(150);
  t2.moveToThread(&s_thread);
  QObject::connect(&s_thread, &QThread::finished, &t2, &QTimer::stop);
  QObject::connect(&t2, &QTimer::timeout,
    [&s]() -> void {
      static Derived d{};
      d.age += 1;      // husband is moving forward in time
      d.wife_age -= 1; // wife is moving backward in time
      emit s.send_2(d);
    });

  // Stop timer 1 after 500 milliseconds and then start timer 2
  QTimer midpoint;
  midpoint.moveToThread(&s_thread);
  midpoint.setSingleShot(true);
  midpoint.setInterval(500);
  QObject::connect(&s_thread, &QThread::started, &midpoint, qOverload<>(&QTimer::start));
  QObject::connect(&midpoint, &QTimer::timeout, &t1, &QTimer::stop);
  QObject::connect(&midpoint, &QTimer::timeout, &t2, qOverload<>(&QTimer::start));

  // run for 1 second
  s_thread.start();
  r_thread.start();
  QTimer::singleShot(1000, &app, &QCoreApplication::quit);
  auto retval = app.exec();

  // stop threads
  s_thread.wait();
  r_thread.wait();

  return retval;
}
