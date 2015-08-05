#ifndef IMAGEVIEWPORTCONTROLLER_H
#define IMAGEVIEWPORTCONTROLLER_H

#include <QWidget>
#include "imageviewport.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include "../qcustomplot/qcustomplot.h"
#include "qprocess.h"
#include <boost/circular_buffer.hpp>
#include <boost/date_time.hpp>
namespace Ui
{
class ImageViewportController;
}

class ImageViewportController : public imageViewport
{
    Q_OBJECT

public:
    static bool acceptsType(QString dataType);

    explicit ImageViewportController(QWidget *parent = 0, QString topic = "", QString dataType = "");
    ~ImageViewportController();
    virtual void callBack(const sensor_msgs::ImageConstPtr& msg);
public slots:
    virtual void trigger(bool val);

private slots:
    virtual void on_bufferSize_change(int val);
    virtual void on_heartBeat_change(double val);
    virtual void on_btnDisable_clicked();
    virtual void on_btnEnable_clicked();
    virtual void on_ckBWPlot_stateChanged(int state);
    virtual void on_readBWProcess();
    virtual void plotBandwidth(double val, double time);
private:
    //Ui::ImageViewportController *ui;
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;

    dynamic_reconfigure::DoubleParameter    heartBeat_param;
    dynamic_reconfigure::BoolParameter      trigger_param;

    dynamic_reconfigure::Config             conf;
    QString controlName;
    QCustomPlot*                            bandwidthPlot;
    QProcess*                               bwProcess;
    boost::circular_buffer<double>          bwHist;
    boost::circular_buffer<double>          bwTimestamp;
    QTimer*                                 bwTimer;
    boost::posix_time::ptime startTime;
    boost::posix_time::ptime lastTime;
};

#endif // IMAGEVIEWPORTCONTROLLER_H
