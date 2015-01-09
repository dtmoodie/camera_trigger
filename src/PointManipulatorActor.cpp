#include "PointManipulatorActor.h"
#include <vtkArrowSource.h>

PointManipulatorActor::PointManipulatorActor()
{
    vtkSmartPointer<vtkArrowSource> arrow = vtkSmartPointer<vtkArrowSource>::New();

}

void
PointManipulatorActor::SetBoundingBox(boundingBoxActor* bb, vtkIdType ptId)
{

}

void
PointManipulatorActor::Move(const float vec[])
{

}

vtkStandardNewMacro(PointManipulatorActor);
