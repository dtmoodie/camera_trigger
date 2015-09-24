#pragma once


#include "renderview.h"
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

class RenderViewportController: public renderView
{
    Q_OBJECT
public:

    static bool acceptsType(QString dataType);


    explicit RenderViewportController(QWidget* parent = 0, QString topic = "", QString dataType = "");
    ~RenderViewportController();


    void callBack(const sensor_msgs::PointCloud2 &pc2);
private slots:
    void on_btnBuild_clicked();
    void on_dblSearchDist_changed(double val);
    void on_pushButton_clicked();
    //void on_btnAddTrigger_clicked();
    //void on_btnPause_clicked();
    //void on_chkShowModel_stateChanged(int state);

private:
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;

    dynamic_reconfigure::DoubleParameter    heartBeat_param;
    dynamic_reconfigure::BoolParameter      trigger_param;

    dynamic_reconfigure::Config             conf;
    QString controlName;
    size_t frameCount;
    bool modelBuilt;
};
