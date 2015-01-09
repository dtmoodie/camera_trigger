#ifndef BOUDINGBOXDIALOG_H
#define BOUDINGBOXDIALOG_H

#include <QDialog>
#include <boost/shared_ptr.hpp>
#include <boundingBoxActor.h>
#include <mainwindow.h>
#include <QDoubleSpinBox>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
class boundingBoxActor;
namespace Ui {
class boundingBoxDialog;
}



class boundingBoxDialog : public QDialog
{
    Q_OBJECT

public:
    explicit boundingBoxDialog(QWidget *parent = NULL);
    explicit boundingBoxDialog(QWidget *parent, vtkSmartPointer<boundingBoxActor> actor_);
    ~boundingBoxDialog();
    vtkSmartPointer<boundingBoxActor> actor;
    void actorUpdated();
private slots:
    void on_currentIndexChanged(QString name);
    void on_autoTrain_clicked(bool val);
    void on_addTrigger_clicked();
private:
    Ui::boundingBoxDialog *ui;
    std::vector<QLabel*> labels;
    std::vector<QDoubleSpinBox*> spinBoxes;
    std::vector<boost::accumulators::accumulator_set<float, boost::accumulators::features<boost::accumulators::tag::mean, boost::accumulators::tag::variance> > > accumulators;
};

#endif // BOUDINGBOXDIALOG_H
