#ifndef LOCALIZING_FILTER_H
#define LOCALIZING_FILTER_H

#include <QObject>
#include <QDateTime>
#include <QVector3D>
#include <QQuaternion>

class LocalizingFilter : public QObject {
  Q_OBJECT

public:
  LocalizingFilter(QObject *parent = nullptr) : QObject(parent) {}

public slots:
  void filter_reading(QDateTime time, QVector3D position,
                      QQuaternion quat, int sensor);

signals:
  void updated_belief(QVector3D position, QQuaternion quat);
};

#endif // LOCALIZING_FILTER_H
