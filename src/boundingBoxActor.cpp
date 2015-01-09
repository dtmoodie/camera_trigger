#include "boundingBoxActor.h"

#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <QLabel>
#include <vtkLine.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>



bool parameter::evaluate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &idx, float Cx, float Cy, float Cz)
{
    return true;
}
moment::moment(int Px_, int Py_, int Pz_, QString name_)
{
    name = name_;
    Px = Px_;
    Py = Py_;
    Pz = Pz_;
    value = 0;
}

bool
moment::evaluate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &idx, float Cx, float Cy, float Cz)
{
    value = 0;
    for(int i = 0; i < idx.size(); ++i)
    {
        value += pow(cloud->points[idx[i]].x - Cx,Px) * pow(cloud->points[idx[i]].y - Cy,Py) * pow(cloud->points[idx[i]].z - Cz,Pz);
    }
    value /= idx.size();
    return true;
}

void
trigger::evaluate(std::vector<boost::shared_ptr<parameter> > parameters)
{
    if(parameters.size() != threshold.size())
        return;
    for(int i = 0; i < threshold.size(); ++i)
    {
        if(parameters[i]->value > threshold[i])
        {
            if(port)
                port->trigger(true);
            return;
        }
    }
    if(port)
        port->trigger(false);
}

boundingBoxActor::boundingBoxActor():
    vtkOpenGLActor()
{
    // This is where all the magic happens of setting things up
    points = vtkSmartPointer<vtkPoints>::New();
    // Create the topology of the point (a vertex)
    vertices =  vtkSmartPointer<vtkCellArray>::New();
    vtkIdType pid[9];
    // Coordinate system is X to the right, Y down, Z forward
    // Start at the top of the box with the origin
    pid[0] = points->InsertNextPoint(0,0,0);
    pid[1] = points->InsertNextPoint(0,0,1);
    pid[2] = points->InsertNextPoint(1,0,1);
    pid[3] = points->InsertNextPoint(1,0,0);

    pid[4] = points->InsertNextPoint(0,1,0);
    pid[5] = points->InsertNextPoint(0,1,1);
    pid[6] = points->InsertNextPoint(1,1,1);
    pid[7] = points->InsertNextPoint(1,1,0);

    pid[8] = points->InsertNextPoint(.5,.5,.5);
    vertices->InsertNextCell(9,pid);

    /*
     *      1          line1                2
     *      |------------------------------
     *      |                               |
     *      Z line0                         | line2
     *      |                               |
     *      |                               |
     *      0--------------- X -------------3
     *                  line3
     * */
    vtkSmartPointer<vtkCellArray> lines =  vtkSmartPointer<vtkCellArray>::New();
    for(int i = 1; i <= 8; ++i)
    {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0,i-1);
        if(i % 4 == 0)
        {
            if(i==4)
                line->GetPointIds()->SetId(1,0);
            else
                line->GetPointIds()->SetId(1,4);
        }
        else
            line->GetPointIds()->SetId(1,i);
        lines->InsertNextCell(line);
    }
    for(int i = 0; i < 4; ++i)
    {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0,i);
        line->GetPointIds()->SetId(1,i+4);
        lines->InsertNextCell(line);
    }

    // Create a polydata object
    polyData = vtkSmartPointer<vtkPolyData>::New();

    polyData->SetLines(lines);
    // Set the points and vertices we created as the geometry and topology of the polydata
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter  = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetTransform(transform);
    //transformFilter->SetInputConnection(polyData);
    transformFilter->SetInputData(polyData);

    // Visualize
    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    //mapper->SetInputData(polyData);
    mapper->SetInputConnection(transformFilter->GetOutputPort());
    SetMapper(mapper);
    GetProperty()->SetPointSize(10);
    GetProperty()->GetColor(red,green,blue);
    red     = double(rand())/RAND_MAX;
    green   = double(rand())/RAND_MAX;
    blue    = double(rand())/RAND_MAX;
    GetProperty()->SetColor(red,green,blue);
    myLabel = new QLabel;
    myLabel->setMinimumHeight(20);
    QColor color(255*red,255*green,255*blue);
    myLabel->setAutoFillBackground(true);
    QPalette p(myLabel->palette());
    p.setColor(QPalette::Background, color);
    myLabel->setPalette(p);
    myLabel->setText("Trigger Region");
}
bool
boundingBoxActor::processPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    boundingBox bb = getBoundingBox();
    std::vector<int> idx;
    for(int i = 0; i < cloud->points.size(); ++i)
    {
        if(bb.in(cloud->points[i]))
            idx.push_back(i);
    }
    if(idx.size() == 0)
        return false;
    CX = CY = CZ = 0;
    for(int i = 0; i < idx.size(); ++i)
    {
        CX += cloud->points[idx[i]].x;
        CY += cloud->points[idx[i]].y;
        CZ += cloud->points[idx[i]].z;
    }
    CX /= idx.size();
    CY /= idx.size();
    CZ /= idx.size();
    for(int i = 0; i < parameters.size(); ++i)
    {
        parameters[i]->evaluate(cloud, idx, CX, CY,CZ);
    }
    // All parameters have been evaluated, now for each trigger, evaluate and trigger accordingly
    for(int i = 0; i < triggers.size(); ++i)
    {
        triggers[i].evaluate(parameters);
    }
    return false;
}

void
boundingBoxActor::selectPoint(vtkIdType id)
{
    // Add arrows to allow dragging of point

}
boundingBox
boundingBoxActor::getBoundingBox()
{
    boundingBox bb;
    bb.maxY = bb.maxZ = bb.maxX = -10000;
    bb.minY = bb.minZ = bb.minX =  10000;
    double pt[3];
    for(vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
    {
        points->GetPoint(i,pt);
        if(pt[0] > bb.maxX) bb.maxX = pt[0];
        if(pt[0] < bb.minX) bb.minX = pt[0];
        if(pt[1] > bb.maxY) bb.maxY = pt[1];
        if(pt[1] < bb.minY) bb.minY = pt[1];
        if(pt[2] > bb.maxZ) bb.maxZ = pt[2];
        if(pt[2] < bb.minZ) bb.minZ = pt[2];
    }
    return bb;
}

bool
boundingBox::in(pcl::PointXYZ pt)
{
    return (pt.x < maxX && pt.x > minX && pt.y < maxY && pt.y > minY && pt.z < maxZ && pt.z > minZ);
}

vtkStandardNewMacro(boundingBoxActor);
