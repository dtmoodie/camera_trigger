#ifndef VIEWPORT_H
#define VIEWPORT_H

#include <QWidget>
#include <ros/ros.h>


namespace Ui {
class viewPort;
}




class viewPort : public QWidget
{
    Q_OBJECT

public:
    static viewPort* newViewport(QWidget* parent, QString topic, QString dataType);

    explicit viewPort(QWidget *parent = 0, QString topic = "", QString dataType = "");

    ~viewPort();

protected:
    Ui::viewPort *ui;
    ros::NodeHandle n;

};

#endif // VIEWPORT_H
