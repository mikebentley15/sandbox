#ifndef REGISTRAR_H
#define REGISTRAR_H

#include <QObject>
#include <QDateTime>
#include <QVector3D>
#include <QQuaternion>

class Registrar : public QObject {
  Q_OBJECT

public:
  Registrar(QObject *parent = nullptr) : QObject(parent) {}

public slots:
  void add_robot_pose(QDateTime time, QVector3D position, QQuaternion quat);
  void add_sensor_reading(QDateTime time, int val);

signals:
  void new_registered_pose(QDateTime time, QVector3D position,
                           QQuaternion quat, int sensor);
};

#endif // REGISTRAR_H
