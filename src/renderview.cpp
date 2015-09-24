#include "renderview.h"
#include "ui_renderview.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <boost/algorithm/cxx11/iota.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <boundingBoxActor.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <qlabel.h>
#include "boundingboxdialog.h"
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackball.h>
#include <vtkWorldPointPicker.h>
#include "PointManipulatorActor.h"

vtkStandardNewMacro(customStyle);
vtkStandardNewMacro(InteractorStyle2);

customStyle::customStyle()
{
    actor = NULL;
    pts = NULL;
    pointManipulator = NULL;
    move = false;
    PointPicker = vtkSmartPointer<vtkPointPicker>::New();
}
void
customStyle::OnLeftButtonDown()
{
    int x = this->Interactor->GetEventPosition()[0];
    int y = this->Interactor->GetEventPosition()[1];
    this->FindPokedRenderer(x, y);
    vtkRenderer* ren = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
    this->PointPicker->Pick(x,y, 0, ren);
    id = this->PointPicker->GetPointId();
    if(id >= 0)
    {
        actor = dynamic_cast<boundingBoxActor*>(PointPicker->GetActor());
        if(actor == NULL)
            return vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        int x = this->Interactor->GetEventPosition()[0];
        int y = this->Interactor->GetEventPosition()[1];
        this->Interactor->GetPicker()->Pick(x,y, 0,Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
        this->Interactor->GetPicker()->GetPickPosition(startPos);
        vtkPolyDataMapper* mapper = dynamic_cast<vtkPolyDataMapper*>(actor->GetMapper());
        if(mapper == NULL)
            return;

        pts = dynamic_cast<vtkPolyData*>(mapper->GetInput());
        pointManipulator = vtkSmartPointer<PointManipulatorActor>::New();
        double pt[3];
        pts->GetPoint(id,pt);
        pointManipulator->SetOrigin(pt);
    }
    return vtkInteractorStyleTrackballCamera::OnLeftButtonDown();

}

void
customStyle::OnMouseMove()
{
    if(pts != NULL)
    {
        int x = this->Interactor->GetEventPosition()[0];
        int y = this->Interactor->GetEventPosition()[1];

        this->Interactor->GetPicker()->Pick(x,y, 0,Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
        double picked[3];
        this->Interactor->GetPicker()->GetPickPosition(picked);
        double vec[3];
        vec[0] = picked[0] - startPos[0];
        vec[1] = picked[1] - startPos[1];
        vec[2] = picked[2] - startPos[2];
        std::cout << "Pixel: " << x << " " << y << " XYZ: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;
        startPos[0] = picked[0];
        startPos[1] = picked[1];
        startPos[2] = picked[2];
        picked[0] = startPos[0] + vec[0];
        picked[1] = startPos[1] + vec[1];
        picked[2] = startPos[2] + vec[2];
        //vtkPoints* points = pts->GetPoints();
        vtkPoints* points = actor->polyData->GetPoints();
        if(id == 8)
        {
            if(actor == NULL)
                return;

            // Grabbing from center point, so move all points
            for(vtkIdType i = 0; i < 9; ++i)
            {
                double pt[3];
                points->GetPoint(i,pt);
                pt[0] += vec[0];
                pt[1] += vec[1];
                pt[2] += vec[2];
                points->SetPoint(i, pt);

            }
        }else
        {
            /*
            double pt[3];
            points->GetPoint(id,pt);
            pt[0] += vec[0];
            pt[1] += vec[1];
            pt[2] += vec[2];
            points->SetPoint(id, pt);*/
        }

        points->Modified();
        Interactor->GetRenderWindow()->Render();
        return;
    }
    vtkInteractorStyleTrackballCamera::OnMouseMove();
}
void
customStyle::OnLeftButtonUp()
{
    pts = NULL;
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}


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
    viewPort(parent,topic,dataType),_cloudBuffer(10),
    _topic(topic),
    currentSettingDialog(NULL),
    ui(new Ui::renderView)
{
    ui->setupUi(this);
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    renderWidget = new QVTKWidget(this);
    renderWidget->setMinimumHeight(240);
    renderWidget->setMinimumWidth(320);
    renderWidget->setSizeIncrement(16,16);

    //style = customStyle::New();
    style = customStyle::New();
    renderWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor(renderWidget->GetInteractor (), renderWidget->GetRenderWindow (), style);

    vtkSmartPointer<vtkWorldPointPicker> worldPointPicker =
        vtkSmartPointer<vtkWorldPointPicker>::New();
    renderWidget->GetInteractor()->SetPicker(worldPointPicker);


    viewer->addCoordinateSystem();
    renderWidget->update();
    ui->gridLayout->addWidget(renderWidget,4,0,1,4);
    std::cout << "Subscribing to topic " << topic.toStdString() << std::endl;
    if(topic.size() > 0)
    {

        sub = n.subscribe(topic.toStdString(), 1, &renderView::callBack, this);
        cloud.reset(new pcl::PCLPointCloud2());
        _cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    connect(ui->chkShowModel, SIGNAL(stateChanged(int)), this, SLOT(on_chkShowModel_stateChanged(int)));
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(on_pushBUtton_clicked()));
}

renderView::~renderView()
{
    delete ui;
}
void radiusSearchHelper(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::search::Search<pcl::PointXYZ>::Ptr searchTree, const std::vector<int>& idx, std::vector<unsigned char>* found, int threads, int thread, float radius)
{
    std::vector<int> knn_idx;
    std::vector<float> dist;
    for(int i = thread; i < idx.size(); i+= threads)
    {
        searchTree->radiusSearch(*cloud, idx[i], radius, knn_idx,dist,1);
        (*found)[i] = knn_idx.size();
    }
}

void
renderView::callBack(const sensor_msgs::PointCloud2 &pc2)
{
    static unsigned int frameCount = 0;
    boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::time_duration delta = currentTime - lastTime;
    //std::fprintf(stdout, "Stream FPS: %02.2f \n", 1000.0/delta.total_milliseconds());
    pcl::fromROSMsg(pc2, *_cloudPtr);
    if(_modelTree)
    {
        boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();

#if USING_GPU
        // GPU
        pcl::gpu::Octree::PointCloud d_cloud;
        d_cloud.upload(_cloudPtr->points);
        pcl::gpu::NeighborIndices neighbors(d_cloud.size(), 1);
        _modelTree->radiusSearch(d_cloud,ui->dblSearchDist->value(),1,neighbors);
        std::vector<int> h_idx, h_sizes;
        neighbors.data.download(h_idx);
        neighbors.sizes.download(h_sizes);
        const int numPoints = std::accumulate(h_sizes.begin(), h_sizes.end(), 0);
        _filteredCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>());
        _filteredCloudPtr->points.reserve(_cloudPtr->points.size() - numPoints);
        for(int i = 0; i < h_sizes.size(); ++i)
        {
            // No neighbor found in the model, push this sucker into the filtered cloud as something out of the range of the model
            if(h_sizes[i] == 0)
            {
                _filteredCloudPtr->points.push_back(_cloudPtr->points[i]);
            }else
            {
                continue;
            }
        }
#else // USING_GPU
    // CPU
        std::vector<int> idx;

        idx.reserve(_cloudPtr->points.size());
        //boost::algorithm::iota(idx.begin(), idx.end(),0);
        for(int i = _cloudPtr->points.size() - 1; i > 0; --i)
            if(_cloudPtr->points[i].x == _cloudPtr->points[i].x)
                idx.push_back(i);
#if USING_KDTREE
        int count = 0;
#if MULTI_THREAD
        std::vector<unsigned char> found(idx.size(),0);
        boost::thread threads[NUM_THREADS];
        for(int i = 0; i < NUM_THREADS; ++i)
        {
            threads[i] = boost::thread(boost::bind(&radiusSearchHelper,_cloudPtr, _modelTree, idx, &found, NUM_THREADS, i, ui->dblSearchDist->value()));
        }
        for(int i = 0; i < NUM_THREADS; ++i)
        {
            threads[i].join();
        }
        count = std::accumulate(found.begin(), found.end(), 0);
        _filteredCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>());
        _filteredCloudPtr->points.reserve(_cloudPtr->points.size() - count);
        for(int i = 0; i < found.size(); ++i)
        {
            if(found[i] == 0)
                _filteredCloudPtr->points.push_back(_cloudPtr->points[idx[i]]);
        }
#else
        std::vector<std::vector<int> > knn_idx;
        std::vector<std::vector<float> > knn_dist;
        _modelTree->radiusSearch(*_cloudPtr, idx, ui->dblSearchDist->value(), knn_idx, knn_dist, 1);

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
#endif

#else // USING_KDTREE
        std::vector<int> knn_idx;
        std::vector<float> knn_dist;
        std::vector<int> filteredIdx;
        for(int i = 0; i < idx.size(); ++i)
        {
            _modelTree->radiusSearch(*_cloudPtr,idx[i], ui->dblSearchDist->value(),knn_idx,knn_dist,1);
            if(knn_idx.size() == 0)
                filteredIdx.push_back(idx[i]);

        }
        int count = filteredIdx.size();
        _filteredCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>());
        _filteredCloudPtr->points.reserve(count);
        for(int i = 0; i < filteredIdx.size(); ++i)
        {
            _filteredCloudPtr->push_back(_cloudPtr->points[filteredIdx[i]]);
        }
#endif // USING_KDTREE
#endif // USING_GPU
        boost::posix_time::ptime end = boost::posix_time::microsec_clock::universal_time();
        boost::posix_time::time_duration delta = end - start;
        std::fprintf(stdout, "Filter time: %4u ms - Point count (remaining/original) %06u / %06u Datarate: %02.2f MB/s \n",
                     delta.total_milliseconds(), _filteredCloudPtr->points.size(), _cloudPtr->points.size(), float(_filteredCloudPtr->points.size())*.01144/delta.total_milliseconds());
        _cloudPtr = _filteredCloudPtr;

        for(int i = 0; i < _boundingBoxes.size(); ++i)
        {
            _boundingBoxes[i]->processPoints(_cloudPtr);
            if(currentSettingDialog)
                if(currentSettingDialog->actor == _boundingBoxes[i])
                    currentSettingDialog->actorUpdated();
        }

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
    delta = currentTime - lastSaveTime;
    if(delta.total_seconds() > 1)
    {
        _cloudBuffer.push_back(_cloudPtr);
        _cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>());
        lastSaveTime = currentTime;
    }
    lastTime = currentTime;
}
bool
renderView::eventFilter(QObject *obj, QEvent *ev)
{
    for(int i = 0; i < _boundingBoxes.size(); ++i)
    {
        if(_boundingBoxes[i]->myLabel == obj)
        {
            if(ev->type() == QEvent::MouseButtonDblClick)
            {
                // Launch a dialog for this trigger region
                if(currentSettingDialog)
                    delete currentSettingDialog;
                currentSettingDialog = new boundingBoxDialog(this, _boundingBoxes[i]);
                currentSettingDialog->show();
            }
        }
    }
}

void
renderView::on_btnBuild_clicked()
{
     // Build a model based off of this snapshot
    _modelPtr = _cloudPtr;
    _cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZ>);
#if USING_KDTREE
    #if USING_GPU


    #else
        _modelTree.reset(new pcl::search::FlannSearch<pcl::PointXYZ>(true));
        _modelTree->setInputCloud(_modelPtr);
    #endif

#else // Octree code
    #if USING_GPU
        _modelTree.reset(new pcl::gpu::Octree());
        pcl::gpu::Octree::PointCloud d_cloud;
        d_cloud.upload(_cloudPtr->points);
        _modelTree->setCloud(d_cloud);
        _modelTree->build();
    #else
        _modelTree.reset(new pcl::search::Octree<pcl::PointXYZ>(ui->dblSearchDist->value()/10));
        _modelTree->setInputCloud(_modelPtr);
    #endif
#endif
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(_modelPtr, 0, 0, 255);
    viewer->removePointCloud("model");
    viewer->addPointCloud(_modelPtr, single_color, "model");
}

void
renderView::on_btnAddTrigger_clicked()
{
    vtkSmartPointer<boundingBoxActor> actor = vtkSmartPointer<boundingBoxActor>::New();
    viewer->addActorToRenderer(actor);
    actor->myLabel->setParent(this);
    ui->triggerLayout->addWidget(actor->myLabel);
    actor->myLabel->show();
    actor->myLabel->installEventFilter(this);
    actor->parameters.push_back(boost::shared_ptr<parameter>(new moment(2,0,0,"U200")));
    actor->parameters.push_back(boost::shared_ptr<parameter>(new moment(0,2,0,"U020")));
    actor->parameters.push_back(boost::shared_ptr<parameter>(new moment(0,0,2,"U002")));
    _boundingBoxes.push_back(actor);
}
void
renderView::on_btnPause_clicked()
{
    static bool active = true;
    if(active == true)
    {
        sub.shutdown();
        ui->btnPause->setText("Resume");
    }else
    {
        sub = n.subscribe(_topic.toStdString(), 1, &renderView::callBack, this);
        ui->btnPause->setText("Pause");
    }
    active = !active;
}


void
renderView::on_chkShowModel_stateChanged(int state)
{
    if(state == 0)
    {
        viewer->removePointCloud("model");
    }else
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(_modelPtr, 0, 0, 255);
        viewer->removePointCloud("model");
        if(_modelPtr != nullptr)
            viewer->addPointCloud(_modelPtr, single_color, "model");
    }
}
void renderView::on_pushBUtton_clicked()
{
    _modelTree.reset();
    viewer->removePointCloud("model");
}
