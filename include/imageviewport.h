#ifndef IMAGEVIEWPORT_H
#define IMAGEVIEWPORT_H

#include <QWidget>
#include "viewport.h"
#include "image_transport/image_transport.h"
#include <QStringList>
namespace Ui {
class imageViewport;
}


class imageViewport : public viewPort
{
    Q_OBJECT
public:
    static bool acceptsType(QString dataType);

    explicit imageViewport(QWidget *parent = 0, QString topic = "", QString dataType = "");
    ~imageViewport();

    void callBack(const sensor_msgs::ImageConstPtr& msg);
private:
    Ui::imageViewport *ui;
    image_transport::ImageTransport  it;
    image_transport::Subscriber sub;

};

#endif // IMAGEVIEWPORT_H
