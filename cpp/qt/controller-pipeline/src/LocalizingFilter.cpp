#include "LocalizingFilter.h"

#include <QtDebug>

/**= Public Slots =**/

void LocalizingFilter::filter_reading(QDateTime time, QVector3D position,
                                      QQuaternion quat, int sensor)
{
  //qDebug() << "LocalizingFilter::filter_reading("
  //         << time << ", " << position << ", " << quat << ", "
  //         << sensor << ")";
  emit updated_belief(position / 2.0, quat);
}

