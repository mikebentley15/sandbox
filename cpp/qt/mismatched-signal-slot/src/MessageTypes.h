#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <QtDebug>
#include <QString>

struct Base {
  QString name = "Mike";
  int age = 33;

  virtual void print_me(const QString &prefix) const {
    qDebug() << prefix << "Base(" << name << "," << age << ")";
  }
};

struct Derived : public Base {
  QString wife = "Sammy";
  int wife_age = 32;

  virtual void print_me(const QString &prefix) const override {
    qDebug() << prefix
             << "Derived(" << name << "," << age << ","
             << wife << "," << wife_age << ")";
  }
};

#endif // MESSAGE_TYPES_H
