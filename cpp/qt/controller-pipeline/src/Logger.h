#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QDateTime>
#include <QVector3D>
#include <QQuaternion>

class Logger : public QObject {
  Q_OBJECT

public:
  Logger(QObject *parent = nullptr) : QObject(parent) {}

public slots:
  void log_robot_pose(QDateTime time, QVector3D position, QQuaternion quat);
  void log_sensor_reading(QDateTime time, int val);
  void log_registered_pose(QDateTime time, QVector3D position,
                           QQuaternion quat, int sensor);
  void log_belief(QVector3D position, QQuaternion quat);
};

#endif // LOGGER_H
