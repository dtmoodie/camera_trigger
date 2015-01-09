#pragma once


#include "vtkOpenGLActor.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <QString>
#include <viewport.h>
class vtkPolyData;
class vtkCellArray;
class vtkPoints;
class vtkPolyDataMapper;
class QLabel;
struct boundingBox
{
    double maxX;
    double minX;
    double maxY;
    double minY;
    double maxZ;
    double minZ;
    bool in(pcl::PointXYZ pt);
};
struct parameter
{
    virtual bool evaluate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& idx, float Cx, float Cy, float Cz);
    float value;
    QString name;
    bool enabled;
};
struct moment: public parameter
{
    moment(int Px_ = 2, int Py_ = 0, int Pz_ = 0, QString name_ = "");
    bool evaluate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &idx, float Cx, float Cy, float Cz);
    int Px, Py, Pz;

};
struct trigger
{
    std::vector<float> threshold;
    void evaluate(std::vector<boost::shared_ptr<parameter> > parameters);
    viewPort* port;
};

class boundingBoxActor: public vtkOpenGLActor
{
public:
    static boundingBoxActor* New();
    boundingBoxActor();
    bool processPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    vtkTypeRevisionMacro(boundingBoxActor, vtkOpenGLActor)
    void selectPoint(vtkIdType id);
    boundingBox getBoundingBox();
    vtkSmartPointer<vtkPolyData> polyData;
    vtkSmartPointer<vtkCellArray> vertices;
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    double red, green, blue;
    QLabel* myLabel;
    bool evaluateRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& idx);
    std::vector<boost::shared_ptr<parameter> > parameters;
    std::vector<trigger> triggers;
    float CX, CY, CZ;
};
vtkCxxRevisionMacro(boundingBoxActor, "$Revision: 1.1 $")
