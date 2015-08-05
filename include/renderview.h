#ifndef RENDERVIEW_H
#define RENDERVIEW_H
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>
#include <viewport.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/subscriber.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <boost/circular_buffer.hpp>
#include <vtkInteractorStyleFlight.h>
#include "vtkObjectFactory.h"
#include <vtkPointPicker.h>
#include <vtkRenderWindow.h>
#include "boundingboxdialog.h"
#include "vtkInteractorStyleTrackballActor.h"
#include <vtkVertexGlyphFilter.h>

#define USING_GPU 0
#define USING_KDTREE 1
#define MULTI_THREAD 1
#define NUM_THREADS 8
#if USING_GPU
#include <pcl/gpu/octree/octree.hpp>
#endif


class PointManipulatorActor;
class boundingBoxActor;

class customStyle: public vtkInteractorStyleTrackballCamera
{
public:
    static customStyle* New();
    customStyle();

    vtkTypeRevisionMacro(customStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnLeftButtonDown();
    virtual void OnLeftButtonUp();
    virtual void OnMouseMove();

    virtual void addActor(vtkSmartPointer<vtkActor> newActor)
    { actors.push_back(newActor);}

    std::vector<vtkSmartPointer<vtkActor> > actors;
    vtkSmartPointer<vtkPointPicker> PointPicker;
    bool move;
    vtkPolyData* pts;
    vtkIdType id;
    double startPos[3];
    vtkSmartPointer<PointManipulatorActor> pointManipulator;
    boundingBoxActor* actor;
};
vtkCxxRevisionMacro(customStyle, "$Revision: 1.1 $")

class InteractorStyle2 : public vtkInteractorStyleTrackballActor
{
  public:
    static InteractorStyle2* New();
    vtkTypeMacro(InteractorStyle2,vtkInteractorStyleTrackballActor);

    InteractorStyle2()
    {
      this->Move = false;
      this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();

      // Setup ghost glyph
      vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();
      points->InsertNextPoint(0,0,0);
      this->MovePolyData = vtkSmartPointer<vtkPolyData>::New();
      this->MovePolyData->SetPoints(points);
      this->MoveGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
      this->MoveGlyphFilter->SetInputConnection(
        this->MovePolyData->GetProducerPort());
#else
      this->MoveGlyphFilter->SetInputData(this->MovePolyData);
#endif
      this->MoveGlyphFilter->Update();

      this->MoveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      this->MoveMapper->SetInputConnection(this->MoveGlyphFilter->GetOutputPort());

      this->MoveActor = vtkSmartPointer<vtkActor>::New();
      this->MoveActor->SetMapper(this->MoveMapper);
      this->MoveActor->VisibilityOff();
      this->MoveActor->GetProperty()->SetPointSize(10);
      this->MoveActor->GetProperty()->SetColor(1,0,0);
    }

    void OnMouseMove()
    {
      if(!this->Move)
        {
        return;
        }

      vtkInteractorStyleTrackballActor::OnMouseMove();

    }

    void OnMiddleButtonUp()
    {
      this->EndPan();

      this->Move = false;
      this->MoveActor->VisibilityOff();

      this->Data->GetPoints()->SetPoint(this->SelectedPoint, this->MoveActor->GetPosition());
      this->Data->Modified();
      this->GetCurrentRenderer()->Render();
      this->GetCurrentRenderer()->GetRenderWindow()->Render();

    }
    void OnMiddleButtonDown()
    {
        // Get the selected point
        int x = this->Interactor->GetEventPosition()[0];
        int y = this->Interactor->GetEventPosition()[1];
        this->FindPokedRenderer(x, y);

        this->PointPicker->Pick(this->Interactor->GetEventPosition()[0],
        this->Interactor->GetEventPosition()[1],
        0,  // always zero.
        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

        if(this->PointPicker->GetPointId() >= 0)
        {
            boundingBoxActor* actor = dynamic_cast<boundingBoxActor*>(PointPicker->GetActor());
            if(actor != NULL)
            {
                vtkPolyDataMapper* mapper = dynamic_cast<vtkPolyDataMapper*>(actor->GetMapper());
                this->Data = mapper->GetInput();
                this->StartPan();
                this->MoveActor->VisibilityOn();
                this->Move = true;
                this->SelectedPoint = this->PointPicker->GetPointId();

                std::cout << "Dragging point " << this->SelectedPoint << std::endl;

                double p[3];
                this->Data->GetPoint(this->SelectedPoint, p);
                std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
                this->MoveActor->SetPosition(p);

                this->GetCurrentRenderer()->AddActor(this->MoveActor);
                this->InteractionProp = this->MoveActor;
            }

        }
    }

  vtkPolyData* Data;
  vtkPolyData* GlyphData;

  vtkSmartPointer<vtkPolyDataMapper> MoveMapper;
  vtkSmartPointer<vtkActor> MoveActor;
  vtkSmartPointer<vtkPolyData> MovePolyData;
  vtkSmartPointer<vtkVertexGlyphFilter> MoveGlyphFilter;

  vtkSmartPointer<vtkPointPicker> PointPicker;

  bool Move;
  vtkIdType SelectedPoint;
};



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
    bool eventFilter(QObject *obj, QEvent *ev);

    void callBack(const sensor_msgs::PointCloud2 &pc2);
protected:

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

#if USING_KDTREE
    #if USING_GPU

    #else
        pcl::search::FlannSearch<pcl::PointXYZ>::Ptr _modelTree;
    #endif
#else
    #if USING_GPU
        pcl::gpu::Octree::Ptr                           _modelTree;
    #else
        pcl::search::Octree<pcl::PointXYZ>::Ptr          _modelTree;
    #endif

#endif
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr   _modelTree;
    boost::circular_buffer< pcl::PointCloud<pcl::PointXYZ>::Ptr> _cloudBuffer;
    boost::posix_time::ptime lastTime;
    boost::posix_time::ptime lastSaveTime;
    QString _topic;
    //customStyle* style;
    vtkInteractorStyle* style;
    std::vector<vtkSmartPointer<boundingBoxActor> > _boundingBoxes;
    boundingBoxDialog*            currentSettingDialog;
private slots:
    virtual void on_btnBuild_clicked();
    virtual void on_btnAddTrigger_clicked();
    virtual void on_btnPause_clicked();
    virtual void on_chkShowModel_stateChanged(int state);
};

#endif // RENDERVIEW_H
