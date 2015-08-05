#include "Imageviewportcontroller.h"
#include "ui_imageviewport.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <boost/date_time.hpp>

static const QStringList TYPES(QStringList() << "sensor_msgs/Image" << "theora_image_transport/Packet");
bool ImageViewportController::acceptsType(QString dataType)
{
    for(int i = 0; i < TYPES.size(); ++i)
    {
        if(dataType == TYPES[i])
            return true;
    }
    return false;
}
ImageViewportController::ImageViewportController(QWidget *parent, QString topic, QString dataType) :
    imageViewport(parent, topic, dataType),
    bwHist(30),
    bwTimestamp(30)
    //ui(new Ui::ImageViewportController)
{
    startTime = boost::date_time::microsec_clock<boost::posix_time::ptime>::universal_time();
    bandwidthPlot = NULL;
    bwProcess = NULL;
    bwTimer = NULL;
    //ui->setupUi(this);
    QStringList tokens = topic.split("/");
    controlName = "/" + tokens[1] + "/set_parameters";
    connect(ui->bufferSize, SIGNAL(valueChanged(int)), this, SLOT(on_bufferSize_change(int)));
    connect(ui->heartBeat, SIGNAL(valueChanged(double)), this, SLOT(on_heartBeat_change(double)));
    connect(ui->btnDisable, SIGNAL(clicked()), this, SLOT(on_btnDisable_clicked()));
    connect(ui->btnEnable, SIGNAL(clicked()), this, SLOT(on_btnEnable_clicked()));
    connect(ui->ckBWPlot, SIGNAL(stateChanged(int)), this, SLOT(on_ckBWPlot_stateChanged(int)));
    _active = -1;
    ui->title->setText(topic);

    // Find the reconfigure topics

}

ImageViewportController::~ImageViewportController()
{
}
void ImageViewportController::callBack(const sensor_msgs::ImageConstPtr& msg)
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
    if(writer == NULL)
    {
        std::string path = _topic.replace('/', '_').toStdString();
        writer = new cv::VideoWriter("/home/dan/build/"+path + ".avi", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, img.size(), img.channels() == 3);
    }
    static int frameCount = 0;
    boost::posix_time::ptime thisTime = boost::date_time::microsec_clock<boost::posix_time::ptime>::universal_time();
    boost::posix_time::time_duration delta = thisTime - lastTime;
    lastTime = thisTime;
    double bandwidth = double(img.rows*img.cols*img.channels()) / (1024.0*1024.0*double(delta.total_milliseconds())/1000);
    boost::posix_time::time_duration elapsedTime = thisTime - startTime;
    double time = elapsedTime.total_milliseconds();
    std::stringstream ss;
    ss << "Elapsed time: " << delta.total_milliseconds();
    cv::putText(img, ss.str(), cv::Point(20,20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,0,0));


    QImage tmpImg((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);

    QPixmap pixmap = QPixmap::fromImage(tmpImg);
    ui->img->setPixmap(pixmap.scaled(ui->img->width(), ui->img->height()));

    ++frameCount;
    cv::cvtColor(img,img,cv::COLOR_RGB2BGR);
    writer->operator <<(img);


    plotBandwidth(bandwidth, time);
}

void ImageViewportController::trigger(bool val)
{
    dynamic_reconfigure::BoolParameter      trigger_param;
    dynamic_reconfigure::Config             conf;
    trigger_param.value = val;
    trigger_param.name = "Active";
    conf.bools.push_back(trigger_param);
    srv_req.config = conf;
    ros::service::call(controlName.toStdString(), srv_req, srv_resp);
}


void ImageViewportController::on_bufferSize_change(int val)
{
    dynamic_reconfigure::IntParameter      BufferSize_param;
    dynamic_reconfigure::Config             conf;
    BufferSize_param.value = val;
    BufferSize_param.name = "BufferSize";
    conf.ints.push_back(BufferSize_param);
    srv_req.config = conf;
    ros::service::call(controlName.toStdString(), srv_req, srv_resp);
}
void ImageViewportController::on_heartBeat_change(double val)
{
    dynamic_reconfigure::DoubleParameter      heartBeat_param;
    dynamic_reconfigure::Config             conf;
    heartBeat_param.value = val;
    heartBeat_param.name = "HeartbeatFrameRate";
    conf.doubles.push_back(heartBeat_param);
    srv_req.config = conf;
    ros::service::call(controlName.toStdString(), srv_req, srv_resp);
}
void ImageViewportController::on_btnDisable_clicked()
{
    trigger(false);
}
void ImageViewportController::on_btnEnable_clicked()
{
    trigger(true);
}
void ImageViewportController::on_ckBWPlot_stateChanged(int state)
{
    if(state == Qt::Checked)
    {
        if(!bandwidthPlot)
        {
            bandwidthPlot = new QCustomPlot(this);
            bandwidthPlot->addGraph();
            ui->gridLayout->addWidget(bandwidthPlot, 4, 2, 1,1);
            bandwidthPlot->setMinimumWidth(320);
            /*bwProcess = new QProcess(this);
            QString application = "rostopic";
            QStringList args;
            args << "bw";
            args << ui->title->text();
            connect(bwProcess, SIGNAL(readyReadStandardOutput()), this, SLOT(on_readBWProcess()));
            connect(bwProcess, SIGNAL(readyReadStandardError()), this, SLOT(on_readBWProcess()));
            bwProcess->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
            bwProcess->setStandardOutputFile("/home/dan/test.txt");
            bwProcess->setProcessChannelMode(QProcess::MergedChannels);
            bwProcess->start(application, args);*/


        }
    }
}
void ImageViewportController::on_readBWProcess()
{
    //QTextStream stream(bwProcess);
    //QString string = stream.readLine();



    QString string(bwProcess->readLine());
    QStringList lines = string.split("\n");
}
void ImageViewportController::plotBandwidth(double val, double time)
{
    if(!bandwidthPlot)
        return;
    bwHist.push_back(val);
    bwTimestamp.push_back(time);
    QVector<double> y;
    y.reserve(bwHist.size());
    QVector<double> x;
    x.reserve(bwHist.size());
    for(int i = 0; i < bwHist.size(); ++i)
    {
        y.push_back(bwHist[i]);
        x.push_back(bwTimestamp[i]);
    }
    double max, min;
    max = *std::max_element(bwTimestamp.begin(), bwTimestamp.end());
    min = *std::min_element(bwTimestamp.begin(), bwTimestamp.end());
    bandwidthPlot->graph(0)->addData(x,y);
    bandwidthPlot->xAxis->setLabel("Time (ms)");
    bandwidthPlot->yAxis->setLabel("Bandwidth (MB/s)");
    bandwidthPlot->xAxis->setRange(min, max);
    bandwidthPlot->yAxis->setRange(0, 30);
    bandwidthPlot->replot();
}
