#pragma once

#include <visualization/Viewer3D.h>
#include <visualization/ViewerColors.h>

#include <vtkImageData.h>
#include <vtkButtonWidget.h>
#include <vtkTexturedButtonRepresentation2D.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

namespace G2D
{
    // TODO: This doesn't work because when you resize the window, button doesn't follow
    void CreateButton(::vtkRenderWindowInteractor* pRenWinInt, ::vtkRenderer* pRen, double x, double y, ViewerColor color)
    {
        // Create the widget and its representation
        // Create two images for texture
        ::vtkSmartPointer<::vtkImageData> image =
                ::vtkSmartPointer<::vtkImageData>::New();

        ::vtkSmartPointer<::vtkTexturedButtonRepresentation2D> buttonRepresentation =
                ::vtkSmartPointer<::vtkTexturedButtonRepresentation2D>::New();
        buttonRepresentation->SetNumberOfStates(2);
        buttonRepresentation->SetButtonTexture(0, image);

        // Specify the size of the image data
        image->SetDimensions(10,10,1);
        #if VTK_MAJOR_VERSION <= 5
        image->SetNumberOfScalarComponents(3);
        image->SetScalarTypeToUnsignedChar();
        #else
        image->AllocateScalars(VTK_UNSIGNED_CHAR,3);
        #endif
        int* dims = image->GetDimensions();

        // Fill the image with
        for (int y = 0; y < dims[1]; y++)
          {
          for (int x = 0; x < dims[0]; x++)
            {
            unsigned char* pixel =
              static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
            if(x<5)
              {
                  pixel[0] = ViewerColors::Banana[0];
                  pixel[1] = ViewerColors::Banana[1];
                  pixel[2] = ViewerColors::Banana[2];
              }
            else
              {
                  pixel[0] = ViewerColors::Tomato[0];
                  pixel[1] = ViewerColors::Tomato[1];
                  pixel[2] = ViewerColors::Tomato[2];
              }
            }
          }

        ::vtkSmartPointer<::vtkButtonWidget> buttonWidget =
                ::vtkSmartPointer<::vtkButtonWidget>::New();
        buttonWidget->SetInteractor(pRenWinInt);
        buttonWidget->SetRepresentation(buttonRepresentation);

        // Place the widget. Must be done after a render so that the
        // viewport is defined.
        // Here the widget placement is in normalized display coordinates
        vtkSmartPointer<vtkCoordinate> upperRight =
          vtkSmartPointer<vtkCoordinate>::New();
        upperRight->SetCoordinateSystemToNormalizedDisplay();
        upperRight->SetValue(1.0, 1.0);

        double bds[6];
        double sz = 50.0;
        bds[0] = upperRight->GetComputedDisplayValue(pRen)[0] - sz;
        bds[1] = bds[0] + sz;
        bds[2] = upperRight->GetComputedDisplayValue(pRen)[1] - sz;
        bds[3] = bds[2] + sz;
        bds[4] = bds[5] = 0.0;

        // Scale to 1, default is .5
        buttonRepresentation->SetPlaceFactor(1);
        buttonRepresentation->PlaceWidget(bds);

        buttonWidget->On();
    }
}
