#include "FakeReader.h"

FakeReader::FakeReader(QObject *parent)
  : QObject(parent)
{
  _timer = std::make_unique<QTimer>(this);
  connect(_timer.get(), &QTimer::timeout,
          this, &FakeReader::create_sensor_reading);
  _timer->start(50);
}


/**= Private Slots =**/

void FakeReader::create_sensor_reading() {
  static int incremental = 0;
  incremental++;
  emit new_sensor_reading(QDateTime::currentDateTime(), incremental);
}
