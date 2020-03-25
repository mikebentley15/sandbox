#ifndef FAKE_READER_H
#define FAKE_READER_H

#include <QObject>
#include <QDateTime>

class FakeReader : public QObject {
  Q_OBJECT

public:
  FakeReader(QObject *parent = nullptr) : QObject(parent) {}

signals:
  void new_sensor_reading(QDateTime time, int val);
};

#endif // FAKE_READER_H
