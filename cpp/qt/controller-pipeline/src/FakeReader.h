#ifndef FAKE_READER_H
#define FAKE_READER_H

#include <QObject>
#include <QDateTime>
#include <QTimer>

#include <memory>

class FakeReader : public QObject {
  Q_OBJECT

public:
  FakeReader(QObject *parent = nullptr);

signals:
  void new_sensor_reading(QDateTime time, int val);

private slots:
  void create_sensor_reading();

private:
  std::unique_ptr<QTimer> _timer;
};

#endif // FAKE_READER_H
