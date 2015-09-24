#include "imageviewport.h"
#include "ui_imageviewport.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/date_time.hpp>


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
    ui(new Ui::imageViewport),
    image_buffer(30),
    writer(NULL)
{
    ui->setupUi(this);
    sub = it.subscribe(topic.toStdString(),1, &imageViewport::callBack,this);
    connect(ui->bufferSize, SIGNAL(valueChanged(int)), this, SLOT(on_bufferSize_change(int)));
    _active = -1;
    ui->title->setText(topic);

}

imageViewport::~imageViewport()
{
    if(writer)
        delete writer;
    delete ui;
}
void
imageViewport::callBack(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("Image received");
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
    if(writer == NULL)
    {
        std::string path = _topic.replace('/', '_').toStdString();
        writer = new cv::VideoWriter("/home/dan/build/"+path + ".avi", CV_FOURCC('X', '2', '6', '4'), 30, img.size(), img.channels() == 3);
    }
    static int frameCount = 0;
    QImage tmpImg((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(tmpImg);
    ui->img->setPixmap(pixmap.scaled(ui->img->width(), ui->img->height()));

    ++frameCount;
    cv::cvtColor(img,img,cv::COLOR_RGB2BGR);
    if(_active > 0)
    {

        writer->operator <<(img);
        --_active;
        return;
    }
    static boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::time_duration delta = currentTime - lastTime;

    if(ui->heartBeat->value() != -1)
    {
        if(ui->heartBeat->value() == 0)
        {
            writer->operator <<(img);
        }else
        {
            if(delta.total_milliseconds() > ui->heartBeat->value() * 1000)
            {
                writer->operator <<(img);
            }
        }
    }
    image_buffer.push_back(img);
}
void
imageViewport::trigger(bool val)
{
    if(_active > 0 && val == true)
    {
        _active = 30;
        return;
    }

    // Dump all images from the buffer
    if(val == true)
    {
        for(boost::circular_buffer<cv::Mat>::iterator it = image_buffer.begin(); it != image_buffer.end(); ++it)
        {
            writer->operator <<(*it);
        }
        _active = 30;
        image_buffer.clear(); // Clear the buffer because we're going to start live streaming for a while
    }
    if(val == false && _active == true)
        _active = 30;
}

void
imageViewport::on_bufferSize_change(int val)
{
    image_buffer.resize(val);
}
