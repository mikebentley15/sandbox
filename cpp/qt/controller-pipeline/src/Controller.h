#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <QDateTime>
#include <QVector3D>
#include <QQuaternion>

class Controller : public QObject {
  Q_OBJECT

public:
  Controller(QObject *parent = nullptr) : QObject(parent) {}

signals:
  void robot_command(QDateTime time, QVector3D position,
                     QQuaternion orientation);
};

#endif // CONTROLLER_H
