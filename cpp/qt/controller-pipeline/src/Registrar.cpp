#include "Registrar.h"

#include <QtDebug>

/**= Public Slots =**/

void Registrar::add_robot_pose(QDateTime time, QVector3D position,
                               QQuaternion quat)
{
  qDebug() << "Registrar::add_robot_pose("
           << time << ", " << position << ", " << quat << ")";
}

void Registrar::add_sensor_reading(QDateTime time, int val) {
  qDebug() << "Registrar::add_sensor_reading("
           << time << ", " << val << ")";
}
