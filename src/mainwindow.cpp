#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QString>
#include <QStringList>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <viewport.h>
void
recursiveAdd(QTreeWidgetItem* item, QStringList topic, QString dataType, int idx)
{
    if(idx > topic.size() - 1) return;
    if(idx == topic.size()-1)
    {
        QTreeWidgetItem* newItem = new QTreeWidgetItem(item);
        newItem->setText(0,topic[idx]);
        newItem->setText(1, dataType);
        return;
    }
    for(int i = 0; i < item->childCount(); ++i)
    {
        if(item->child(i)->text(0) == topic[idx])
        {
            recursiveAdd(item->child(i), topic, dataType, idx+1);
            return;
        }
    }
    QTreeWidgetItem* newItem = new QTreeWidgetItem(item);
    newItem->setText(0,topic[idx]);
    //newItem->setText(1, dataType);
    recursiveAdd(newItem, topic, dataType, idx+1);
}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->topicView, SIGNAL(itemActivated(QTreeWidgetItem*, int)), this, SLOT(on_itemActivated(QTreeWidgetItem*, int)));
    processTimer = new QTimer(this);
    processTimer->start(30);
    connect(processTimer, SIGNAL(timeout()), this, SLOT(on_process()));
}

MainWindow::~MainWindow()
{
    delete ui;
}



void
MainWindow::on_btnRefresh_clicked()
{
    ros::master::V_TopicInfo ti;
    QTreeWidget* tree = ui->topicView;
    if(!ros::master::getTopics(ti))
        return;
    for(ros::master::V_TopicInfo::iterator it = ti.begin(); it != ti.end(); ++it)
    {
        QString name = QString::fromLatin1(it->name.c_str());
        QString dataType = QString::fromStdString(it->datatype);
        QStringList topic = name.split("/");
        bool added = false;
        for(int i = 0; i < tree->topLevelItemCount(); ++i)
        {
            if(tree->topLevelItem(i)->text(0) == topic[1])
            {
                recursiveAdd(tree->topLevelItem(i), topic, dataType, 2);
                added = true;
                continue;
            }
        }
        if(added == true)
            continue;
        // This top level topic doesn't exist yet, add it
        QTreeWidgetItem* item = new QTreeWidgetItem(tree);
        item->setText(0, topic[1]);
        recursiveAdd(item,topic,dataType,2);
    }
}
void
MainWindow::on_itemActivated(QTreeWidgetItem* item, int col)
{
    QTreeWidgetItem* parent = item->parent();
    QString topic = item->text(0);
    while(parent)
    {
        topic = parent->text(0) + "/" + topic;
        parent = parent->parent();
    }
    topic = "/" + topic;
    viewPort* newPort = viewPort::newViewport(this,topic,item->text(1));
    ui->viewportLayout->addWidget(newPort);
}
void
MainWindow::on_process()
{
    ros::spinOnce();
}
