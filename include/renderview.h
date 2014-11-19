#ifndef RENDERVIEW_H
#define RENDERVIEW_H
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <viewport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/subscriber.h>
//#include <pcl/search/octree.h>

//#include <pcl/gpu/octree/octree.hpp>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
namespace Ui {
class renderView;
}

class renderView : public viewPort
{
    Q_OBJECT


public:

    static bool acceptsType(QString dataType);


    explicit renderView(QWidget* parent = 0, QString topic = "", QString dataType = "");
    ~renderView();


    void callBack(const sensor_msgs::PointCloud2 &pc2);
private:

    Ui::renderView *ui;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    QVTKWidget* renderWidget;
    ros::Subscriber sub;
    pcl::PCLPointCloud2Ptr cloud;
    pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr rgb;
    pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::Ptr geometry_handler;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCLPointCloud2ConstPtr tmpCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _filteredCloudPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _modelPtr;
    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr _modelTree;

//    pcl::gpu::Octree::PointCloud                d_modelCloud;
//    pcl::gpu::Octree::Ptr                        _modelTree;
private slots:
    void on_btnBuild_clicked();
    void on_btnAddTrigger_clicked();
};

#endif // RENDERVIEW_H
