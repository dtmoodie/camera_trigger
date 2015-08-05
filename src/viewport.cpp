#include "viewport.h"
#include "ui_viewport.h"

#include <imageviewport.h>
#include <renderview.h>
#include <Imageviewportcontroller.h>
#include <RenderViewportController.h>

viewPort* viewPort::newViewport(QWidget* parent, QString topic, QString dataType)
{
    if(ImageViewportController::acceptsType(dataType))
        return new ImageViewportController(parent,topic,dataType);
    if(imageViewport::acceptsType(dataType))
        return new imageViewport(parent,topic,dataType);
    //if(RenderViewportController::acceptsType(dataType))
      //  return new RenderViewportController(parent,topic,dataType);
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
