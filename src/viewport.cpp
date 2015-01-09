#include "viewport.h"
#include "ui_viewport.h"

#include <imageviewport.h>
#include <renderview.h>



viewPort* viewPort::newViewport(QWidget* parent, QString topic, QString dataType)
{
    if(imageViewport::acceptsType(dataType))
        return new imageViewport(parent,topic,dataType);
    if(renderView::acceptsType(dataType))
        return new renderView(parent,topic,dataType);

    return NULL;
}


viewPort::viewPort(QWidget *parent, QString topic, QString dataType) :
    QWidget(parent),
    ui(new Ui::viewPort)
{
    ui->setupUi(this);
    _topic = topic;
}

viewPort::~viewPort()
{
    delete ui;
}
void
viewPort::trigger(bool val)
{
    return;
}
