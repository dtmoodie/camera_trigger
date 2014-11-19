#include "renderview.h"
#include "ui_renderview.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkRenderWindow.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <boost/algorithm/cxx11/iota.hpp>
#include <stdio.h>


static const QStringList TYPES(QStringList() << "sensor_msgs/PointCloud2");
bool
renderView::acceptsType(QString dataType)
{
    for(int i = 0; i < TYPES.size(); ++i)
    {
        if(dataType == TYPES[i])
            return true;
    }
    return false;
}

renderView::renderView(QWidget* parent, QString topic, QString dataType) :
    viewPort(parent,topic,dataType),
    ui(new Ui::renderView)
{
    ui->setupUi(this);
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    renderWidget = new QVTKWidget(this);
    renderWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor(renderWidget->GetInteractor (), renderWidget->GetRenderWindow ());
    viewer->addCoordinateSystem();
    renderWidget->update();
    ui->gridLayout->addWidget(renderWidget,2,0,1,2);
    sub = n.subscribe(topic.toStdString(), 1, &renderView::callBack, this);
    cloud.reset(new pcl::PCLPointCloud2());
    _cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

renderView::~renderView()
{
    delete ui;
}
void
renderView::callBack(const sensor_msgs::PointCloud2 &pc2)
{
    static unsigned int frameCount = 0;
    pcl::fromROSMsg(pc2, *_cloudPtr);
    if(_modelTree)
    {
        boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
        std::vector<int> idx;
        idx.reserve(_cloudPtr->points.size());
        //boost::algorithm::iota(idx.begin(), idx.end(),0);
        for(int i = _cloudPtr->points.size() - 1; i > 0; --i)
            if(_cloudPtr->points[i].x == _cloudPtr->points[i].x)
                idx.push_back(i);
        std::vector<std::vector<int> > knn_idx;
        std::vector<std::vector<float> > knn_dist;
        _modelTree->radiusSearch(*_cloudPtr, idx, ui->dblSearchDist->value(), knn_idx, knn_dist, 1);
        int count = 0;
        for(int i = 0; i < knn_idx.size(); ++i)
            count += knn_idx[i].size();
        // Count is the number of points that lie on the model
        _filteredCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>());
        _filteredCloudPtr->points.reserve(_cloudPtr->points.size() - count);
        for(int i = 0; i < knn_idx.size(); ++i)
        {
            if(knn_idx[i].size() == 0)
            {
                _filteredCloudPtr->points.push_back(_cloudPtr->points[i]);
            }
        }
        boost::posix_time::ptime end = boost::posix_time::microsec_clock::universal_time();
        boost::posix_time::time_duration delta = end - start;
        std::fprintf(stdout, "Filter time: %04u - Point count (remaining/original) %06u / %06u Datarate: %02.2f MB/s \n", delta.total_milliseconds(),_filteredCloudPtr->points.size(),_cloudPtr->points.size(), float(_filteredCloudPtr->points.size())*.01144/delta.total_milliseconds());
        //std::cout << "Filtering took: " << delta.total_milliseconds() << " milliseconds. Resulting points (remaining / original): " << _filteredCloudPtr->points.size() << " / " << _cloudPtr->points.size() << std::endl;
        _cloudPtr = _filteredCloudPtr;
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

void
renderView::on_btnBuild_clicked()
{
     // Build a model based off of this snapshot
    _modelPtr = _cloudPtr;
    _cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    _modelTree.reset(new pcl::search::FlannSearch<pcl::PointXYZ>(true));
    _modelTree->setInputCloud(_modelPtr);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(_modelPtr, 0, 0, 255);
    viewer->removePointCloud("model");
    viewer->addPointCloud(_modelPtr, single_color, "model");
}

void
renderView::on_btnAddTrigger_clicked()
{

}
