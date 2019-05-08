#ifndef CARLASTREAMTHREAD_H
#define CARLASTREAMTHREAD_H

#include <QThread>
#include <QImage>

#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Sensor.h>
#include <carla/Memory.h>
#include <carla/sensor/data/Image.h>

class QImage;

class CarlaStreamThread : public QThread
{
    Q_OBJECT

public:
    CarlaStreamThread(QObject *parent = 0);
    ~CarlaStreamThread();

signals:
    void renderedImage(const QImage &image);

protected:
    void run() Q_DECL_OVERRIDE;

private:

   carla::client::Client client_connection;
   carla::SharedPtr<carla::client::Sensor> camera;

   int image_callback (carla::SharedPtr<carla::sensor::SensorData> data);

};

#endif // CARLASTREAMTHREAD_H
