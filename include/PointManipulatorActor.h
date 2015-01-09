#pragma once

#include <vtkOpenGLActor.h>
#include "vtkObjectFactory.h"
#include <vtkSmartPointer.h>
class boundingBoxActor;

class PointManipulatorActor: public vtkOpenGLActor
{
public:
    static PointManipulatorActor* New();
    vtkTypeRevisionMacro(PointManipulatorActor, vtkOpenGLActor)
    PointManipulatorActor();
    void SetBoundingBox(boundingBoxActor* bb, vtkIdType ptId);

    void Move(const float vec[]);
};
vtkCxxRevisionMacro(PointManipulatorActor, "$Revision: 1.1 $")
