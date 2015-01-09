#include "boundingboxdialog.h"
#include "ui_boundingboxdialog.h"
#include <QCheckBox>
#include <QDoubleSpinBox>

boundingBoxDialog::boundingBoxDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::boundingBoxDialog)
{
    ui->setupUi(this);
}
boundingBoxDialog::boundingBoxDialog(QWidget *parent, vtkSmartPointer<boundingBoxActor> actor_):
    QDialog(parent),
    ui(new Ui::boundingBoxDialog),
    actor(actor_)
{
    ui->setupUi(this);
    for(int i = 0; i < actor->parameters.size(); ++i)
    {
        // For each parameter, spawn a controller object
        QCheckBox* box = new QCheckBox(this);
        box->setText(actor->parameters[i]->name);
        box->setChecked(true);
        QLabel* lbl = new QLabel(this);
        lbl->setText(QString::number(actor->parameters[i]->value));
        QDoubleSpinBox* spinBox = new QDoubleSpinBox(this);
        spinBox->setDecimals(6);
        labels.push_back(lbl);
        spinBoxes.push_back(spinBox);
        ui->parameterLayout->addWidget(box,i,0);
        ui->parameterLayout->addWidget(lbl,i,1);
        ui->parameterLayout->addWidget(spinBox,i,2);
    }
    std::vector<viewPort*> ports = MainWindow::getViewports();
    for(int i = 0; i < ports.size(); ++i)
    {
        ui->receivers->addItem(ports[i]->_topic);
    }
    connect(ui->receivers, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_currentIndexChanged(QString)));
    connect(ui->autoTrain, SIGNAL(clicked(bool)), this, SLOT(on_autoTrain_clicked(bool)));
    connect(ui->addTrigger, SIGNAL(clicked()), this, SLOT(on_addTrigger_clicked()));
}

boundingBoxDialog::~boundingBoxDialog()
{
    delete ui;
}

void
boundingBoxDialog::on_addTrigger_clicked()
{
    // Create a trigger object and add it to this actor
    trigger tmpTrigger;
    std::vector<viewPort*> ports = MainWindow::getViewports();
    for(int i = 0; i < spinBoxes.size(); ++i)
    {
        tmpTrigger.threshold.push_back(spinBoxes[i]->value());
    }
    for(int i = 0; i < ports.size(); ++i)
    {
        if(ports[i]->_topic == ui->receivers->currentText())
            tmpTrigger.port = ports[i];
    }
    actor->triggers.push_back(tmpTrigger);
}

void
boundingBoxDialog::actorUpdated()
{
    ui->centroidDisp->setText("Centroid (x,y,z): " + QString::number(actor->CX) + " " + QString::number(actor->CY) + " " + QString::number(actor->CZ));
    for(int i = 0; i < actor->parameters.size(); ++i)
    {
        labels[i]->setText(QString::number(actor->parameters[i]->value));
        if(accumulators.size() == actor->parameters.size())
        {
            accumulators[i](actor->parameters[i]->value);
        }
    }
}

void
boundingBoxDialog::on_autoTrain_clicked(bool val)
{
    if(val == true)
    {
        accumulators.resize(actor->parameters.size());
    }else
    {
        if(accumulators.size() == actor->parameters.size())
        {
            // Adjust threshold based on mean and variance of the dataset
            for(int i = 0; i < accumulators.size(); ++i)
            {
                float mean = boost::accumulators::mean(accumulators[i]);
                float std = boost::accumulators::variance(accumulators[i]);
                std = sqrt(std);
                // Set the spin box to the value
                spinBoxes[i]->setValue(mean + 2* std);
            }
        }
    }
}

void
boundingBoxDialog::on_currentIndexChanged(QString name)
{
    // Check if a trigger already exists for this item
    for(int i = 0; i < actor->triggers.size(); ++i)
    {
        if(actor->triggers[i].port->_topic == name)
        {
            // This trigger exists, update the threshold values
            if(actor->triggers[i].threshold.size() != actor->parameters.size())
                return;
            for(int j = 0; j < actor->parameters.size(); ++j)
            {
                QLayoutItem * item = ui->parameterLayout->itemAtPosition(j,2);
                QSpinBox* spinBox = dynamic_cast<QSpinBox*>(item);
                if(spinBox == NULL)
                    continue;
                spinBox->setValue(actor->triggers[i].threshold[j]);
            }
        }
    }
}
