#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QDateTime>
#include <QObject>
#include <QQuaternion>
#include <QTimer>
#include <QVector3D>

#include <memory>

class Controller : public QObject {
  Q_OBJECT

public:
  Controller(QObject *parent = nullptr);

signals:
  void robot_command(QDateTime time, QVector3D position,
                     QQuaternion orientation);

private slots:
  void create_next_command();

private:
  std::unique_ptr<QTimer> _timer;
};

#endif // CONTROLLER_H
