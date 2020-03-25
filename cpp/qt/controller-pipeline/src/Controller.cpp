#include "Controller.h"

#include <cmath>  // for M_PI

Controller::Controller(QObject *parent)
  : QObject(parent)
{
  _timer = std::make_unique<QTimer>(this);
  connect(_timer.get(), &QTimer::timeout,
          this, &Controller::create_next_command);
  _timer->start(10);
}


/**= Private Slots =**/

void Controller::create_next_command()
{
  static QVector3D current_pos {};
  static QQuaternion current_quat {};
  const QVector3D delta_pos {0.1, 0.01, 0.001};
  const QQuaternion delta_rot = QQuaternion::fromAxisAndAngle(1, 1, 1, M_PI/60);
  current_pos += delta_pos;
  current_quat *= delta_rot;
  emit robot_command(
      QDateTime::currentDateTime(),
      current_pos, current_quat);
}
