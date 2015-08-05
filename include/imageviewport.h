#ifndef IMAGEVIEWPORT_H
#define IMAGEVIEWPORT_H

#include <QWidget>
#include "viewport.h"
#include "image_transport/image_transport.h"
#include <QStringList>
#include <boost/circular_buffer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace Ui
{
class imageViewport;
}


class imageViewport : public viewPort
{
    Q_OBJECT
public:
    static bool acceptsType(QString dataType);

    explicit imageViewport(QWidget *parent = 0, QString topic = "", QString dataType = "");
    ~imageViewport();

    virtual void callBack(const sensor_msgs::ImageConstPtr& msg);
    virtual void trigger(bool val);

private slots:
    virtual void on_bufferSize_change(int val);
protected:
    Ui::imageViewport *ui;
    image_transport::ImageTransport  it;
    image_transport::Subscriber sub;
    boost::circular_buffer<cv::Mat> image_buffer;
    cv::VideoWriter* writer;

};

#endif // IMAGEVIEWPORT_H
