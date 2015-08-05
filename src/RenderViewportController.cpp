#include "RenderViewportController.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include "ui_renderview.h"

static const QStringList TYPES(QStringList() << "sensor_msgs/PointCloud2");

bool RenderViewportController::acceptsType(QString dataType)
{
    for(int i = 0; i < TYPES.size(); ++i)
    {
        if(dataType == TYPES[i])
            return true;
    }
    return false;
}


RenderViewportController::RenderViewportController(QWidget *parent, QString topic, QString dataType):
    renderView(parent,topic,dataType)
{
    if(topic.size() > 0)
    {
        sub = n.subscribe(topic.toStdString(), 1, &RenderViewportController::callBack, this);
    }
    QStringList tokens = topic.split("/");
    controlName = "/" + tokens[1] + "/set_parameters";
    connect(ui->dblSearchDist, SIGNAL(valueChanged(double)), this, SLOT(on_dblSearchDist_changed(double)));
}
RenderViewportController::~RenderViewportController()
{

}

void RenderViewportController::callBack(const sensor_msgs::PointCloud2 &pc2)
{
    static int frameCount = 0;
    if(_cloudPtr == NULL)
        _cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pc2,*_cloudPtr);
    for(int i = 0; i < _boundingBoxes.size(); ++i)
    {
        _boundingBoxes[i]->processPoints(_cloudPtr);
        if(currentSettingDialog)
            if(currentSettingDialog->actor == _boundingBoxes[i])
                currentSettingDialog->actorUpdated();
    }
    if(frameCount == 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(_cloudPtr, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ>(_cloudPtr, single_color, "cloud");
    }
    else
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(_cloudPtr, 0, 255, 0);
        viewer->updatePointCloud<pcl::PointXYZ>(_cloudPtr, single_color, "cloud");
    }
    ++frameCount;
    renderWidget->update();
}

void RenderViewportController::on_btnBuild_clicked()
{
    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::BoolParameter buildParameter;
    buildParameter.value = true;
    buildParameter.name = "BuildModel";
    conf.bools.push_back(buildParameter);
    srv_req.config = conf;
    ros::service::call(controlName.toStdString(), srv_req, srv_resp);
}

void RenderViewportController::on_dblSearchDist_changed(double val)
{
    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::IntParameter parameter;
    parameter.name = "ThresholdWindow";
    parameter.value = 1000*val;
    conf.ints.push_back(parameter);
    srv_req.config = conf;
    ros::service::call(controlName.toStdString(), srv_req, srv_resp);
}
