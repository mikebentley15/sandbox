#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <QtDebug>
#include <QString>
#include <QThread>

#include <string>
#include <sstream>

inline const char* current_thread_id() {
  static thread_local std::string id;
  if (id.empty()) {
    std::ostringstream out;
    out << "QThread(" << QThread::currentThreadId() << ")";
    id = out.str();
  }
  return id.c_str();
}

struct Base {
  QString name = "Mike";
  int age = 33;

  virtual void print_me(const std::string &prefix) const {
    qDebug() << prefix.c_str() << "Base(" << name << "," << age << ")";
  }
};
Q_DECLARE_METATYPE(Base);

struct Derived : public Base {
  QString wife = "Sammy";
  int wife_age = 32;

  virtual void print_me(const std::string &prefix) const override {
    qDebug() << prefix.c_str()
             << "Derived(" << name << "," << age << ","
             << wife << "," << wife_age << ")";
  }
};
Q_DECLARE_METATYPE(Derived);

#endif // MESSAGE_TYPES_H
