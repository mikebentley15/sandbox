#include "Logger.h"

#include <QtDebug>

/**= Public Slots =**/

void Logger::log_robot_pose(QDateTime time, QVector3D position,
                            QQuaternion quat)
{
  qDebug() << "Logger::log_robot_pose("
           << time << ", " << position << ", " << quat << ")";
}

void Logger::log_sensor_reading(QDateTime time, int val)
{
  qDebug() << "Logger::log_sensor_reading("
           << time << ", " << val << ")";
}

void Logger::log_registered_pose(QDateTime time, QVector3D position,
                                 QQuaternion quat, int sensor)
{
  qDebug() << "Logger::log_registered_pose("
           << time << ", " << position << ", " << quat << ", "
           << sensor << ")";
}

void Logger::log_belief(QVector3D position, QQuaternion quat)
{
  qDebug() << "Logger::log_belief("
           << position << ", " << quat << ")";
}

void Logger::log_quit()
{
  qDebug() << "Logger::log_quit()";
}
