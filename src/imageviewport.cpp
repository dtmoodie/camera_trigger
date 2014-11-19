#include "imageviewport.h"
#include "ui_imageviewport.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


static const QStringList TYPES(QStringList() << "sensor_msgs/Image");
bool imageViewport::acceptsType(QString dataType)
{
    for(int i = 0; i < TYPES.size(); ++i)
    {
        if(dataType == TYPES[i])
            return true;
    }
    return false;
}

imageViewport::imageViewport(QWidget *parent, QString topic, QString dataType) :
    viewPort(parent,topic,dataType),
    it(n),

    ui(new Ui::imageViewport)
{
    ui->setupUi(this);
    sub = it.subscribe(topic.toStdString(),1, &imageViewport::callBack,this);
}

imageViewport::~imageViewport()
{
    delete ui;
}
void
imageViewport::callBack(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat img;
    try
    {
      img = cv_bridge::toCvShare(msg, "rgb8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      return;
    }
    QImage tmpImg((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(tmpImg);
    ui->img->setPixmap(pixmap.scaled(ui->img->width(), ui->img->height()));
}
