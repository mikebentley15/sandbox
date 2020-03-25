#include "Registrar.h"

#include <QtDebug>

/**= Public Slots =**/

void Registrar::add_robot_pose(QDateTime time, QVector3D position,
                               QQuaternion quat)
{
  //qDebug() << "Registrar::add_robot_pose("
  //         << time << ", " << position << ", " << quat << ")";
  _last_pos = position;
  _last_quat = quat;
}

void Registrar::add_sensor_reading(QDateTime time, int val) {
  //qDebug() << "Registrar::add_sensor_reading("
  //         << time << ", " << val << ")";
  emit new_registered_pose(time, _last_pos, _last_quat, val);
}
