#include "visualization/Viewer3D.h"

// For Colored points
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPointData.h>

// For Frustums
#include <vtkDataSetMapper.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>

// For Axes
#include <vtkAxesActor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <functional>
#include <thread>

namespace G2D
{
    class KeyPressInteractorStyle : public ::vtkInteractorStyleTrackballCamera
    {
      public:
        static KeyPressInteractorStyle* New()
        {
            return new KeyPressInteractorStyle;
        }
        vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

        virtual void OnKeyPress()
        {
            // Get the keypress
            vtkRenderWindowInteractor *rwi = this->Interactor;
            std::string key = rwi->GetKeySym();

            if(key == "space")
            {
                std::cout << "[INFO] Viewer3D: Step" << std::endl;
                m_pViewer->Step();
            }
            else if (key == "Return")
            {
                std::cout << "[INFO] Viewer3D: Play" << std::endl;
                m_pViewer->Play();
            }
            // Handle on exit (vtk has no callback registration on exit)
            else if(key == "e")
            {
                // Release the user application
                // TODO: Call a better "terminating" function on viewer?
                std::cout << "[INFO] Viewer3D: Exiting" << std::endl;
                m_pViewer->Play();
            }

            // Forward events
            vtkInteractorStyleTrackballCamera::OnKeyPress();
        }

        Viewer3D* m_pViewer;

    };

    class viewerTimerCallback : public vtkCommand
    {
    public:
        static viewerTimerCallback* New()
        {
            viewerTimerCallback* pCallback = new viewerTimerCallback();
            pCallback->TimerCount = 0;
            return pCallback;
        }

        virtual void Execute(
                vtkObject *caller,
                unsigned long eventId,
                void * vtkNotUsed(callData))
        {
            if (vtkCommand::TimerEvent == eventId)
            {
                this->TimerCount++;
                if (onTimerCallback != nullptr)
                {
                    onTimerCallback(caller, eventId);
                }
            }
        }

    private:
        unsigned long TimerCount;

    public:
        std::function<void(vtkObject*, unsigned long)> onTimerCallback;

        //
    };

    Viewer3D::Viewer3D()
    {
        m_playstate = PLAYSTATE_PAUSE;
    }

    Viewer3D::~Viewer3D()
    {
        this->Play(); // Notify everybody to just play through
        m_pRenderWindowInteractor->TerminateApp();
    }

    void Viewer3D::PolydataAlgoSetInputData(::vtkPolyDataAlgorithm* pPolydata, ::vtkDataObject* pInput)
    {
        #if VTK_MAJOR_VERSION <= 5
        pPolydata->SetInput(pInput);
        #else
        pPolydata->SetInputData(pInput);
        #endif
    }
    void Viewer3D::PolydataMapperSetPolyData(::vtkPolyDataMapper* pMapper, ::vtkPolyData* pPolyData)
    {
        #if VTK_MAJOR_VERSION <= 5
        pMapper->SetInputConnection(pPolyData->GetProducerPort());
        #else
          pMapper->SetInputData(pPolyData);
        #endif
    }
    void Viewer3D::DataSetMapperSetDataSet(::vtkDataSetMapper* pMapper, ::vtkDataSet* pDataSet)
    {
        #if VTK_MAJOR_VERSION <= 5
          pMapper->SetInput(ug);
        #else
          pMapper->SetInputData(pDataSet);
        #endif
    }

    void Viewer3D::SetupVtkColoredPoints()
    {
        // Allocate memory to hold the points
        // Create a polydata object that holds the points
        m_pPoints = vtkSmartPointer<vtkPoints>::New();
        m_pPoints_polydata = vtkSmartPointer<vtkPolyData>::New();
        m_pPoints_polydata->SetPoints(m_pPoints);

        // Create a vertex filter for the points, which will eventually filter the points and add color
        m_pPoints_vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        PolydataAlgoSetInputData(m_pPoints_vertexFilter, m_pPoints_polydata);
        m_pPoints_vertexFilter->Update();

        // Create a polydata object that holds the points, after filtering
        m_pPoints_filteredPolydata = vtkSmartPointer<vtkPolyData>::New();
        m_pPoints_filteredPolydata->ShallowCopy(m_pPoints_vertexFilter->GetOutput());

        // Allocate memory to hold the colors
        m_pPoints_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        m_pPoints_colors->SetNumberOfComponents(3);
        m_pPoints_colors->SetName ("Colors");

        // Map the filtered polydata to an actor, and add to renderer
        vtkSmartPointer<vtkPolyDataMapper> mapper =
          vtkSmartPointer<vtkPolyDataMapper>::New();
        PolydataMapperSetPolyData(mapper, m_pPoints_filteredPolydata);

        vtkSmartPointer<vtkActor> actor =
          vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(5);

        m_pRenderer->AddActor(actor);
    }

    void Viewer3D::SetupVtkPoseFrustums()
    {
        m_pFrustumPoints = vtkSmartPointer<vtkPoints>::New();

        m_arrUnstructuredGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();

        //Create an actor and mapper
        vtkSmartPointer<vtkDataSetMapper> mapper =
            vtkSmartPointer<vtkDataSetMapper>::New();
        DataSetMapperSetDataSet(mapper, m_arrUnstructuredGrid);

        m_pFrustums_actor = vtkSmartPointer<vtkActor>::New();
        m_pFrustums_actor->SetMapper(mapper);

        // Generate a wireframe
        m_pFrustums_actor->GetProperty()->SetRepresentationToWireframe();

        //Create a renderer, render window, and interactor
        m_pRenderer->AddActor(m_pFrustums_actor);
    }

    void Viewer3D::SetupAxes()
    {
          vtkSmartPointer<vtkAxesActor> axes =
            vtkSmartPointer<vtkAxesActor>::New();

          axes->SetNormalizedShaftLength(10, 10, 10);

          m_pRenderer->AddActor(axes);
    }

    void Viewer3D::InitializeAndRun()
    {
        // Create render/window/interactor
        m_pRenderer = ::vtkRenderer::New();
        m_pRenderWindow = ::vtkRenderWindow::New();
        m_pRenderWindow->AddRenderer(m_pRenderer);
        m_pRenderWindowInteractor = ::vtkRenderWindowInteractor::New();
        m_pRenderWindowInteractor->SetRenderWindow(m_pRenderWindow);
        m_pRenderWindowInteractor->Initialize();

        // Create the "actors" (e.g. point clouds, pose frustums, etc.)

        // Add an axes for orientation
        SetupAxes();

        // Add points
        SetupVtkColoredPoints();

        // Add frustums
        SetupVtkPoseFrustums();

        // Make the vtkRenderWindowInteractor poll Viewer3D for updates
        // Do so by creating a periodic timer event, and make Viewer3D's
        // update function an observer of the timer
        viewerTimerCallback* pCallback = viewerTimerCallback::New();
        pCallback->onTimerCallback =
                [&](::vtkObject* caller, unsigned long eventId)
                {
                    this->OnPollUpdate(caller, eventId);
                };
        m_pRenderWindowInteractor->AddObserver(
                ::vtkCommand::TimerEvent,
                pCallback);
        // Fire every 100 ms
        m_pRenderWindowInteractor->CreateRepeatingTimer(100);

        // Set a callback for keyboard commands
        vtkSmartPointer<KeyPressInteractorStyle> style =
                vtkSmartPointer<KeyPressInteractorStyle>::New();
        style->m_pViewer = this;
        m_pRenderWindowInteractor->SetInteractorStyle(style);
        style->SetCurrentRenderer(m_pRenderer);

        // Set some defaults on the render window
        m_pRenderWindow->SetSize(900, 900);
        m_pRenderWindow->SetPosition(900, 0);

        // Start Render
        m_pRenderWindow->Render();

        // Start interaction
        m_pRenderWindowInteractor->Start();
    }

    void Viewer3D::InitializeAndRunAsync()
    {
        m_renderThread = std::thread(&Viewer3D::InitializeAndRun, this);
    }

    void Viewer3D::MaybeYieldToViewer()
    {
        // Wait on the viewer until the user resumes playing
        std::unique_lock<std::mutex> lock(m_playstate_lock);
        m_playstate_changed.wait(
                lock,
                [this]
                {
                    return m_playstate != PLAYSTATE_PAUSE;
                });

        // If user has only asked to step, then set playstate
        // back to a paused
        if (m_playstate == PLAYSTATE_STEP)
        {
            m_playstate = PLAYSTATE_PAUSE;
        }

        // Free the lock and let caller resume work
        lock.unlock();
    }

    void Viewer3D::Play()
    {
        bool playstateChanged = false;
        {
            std::lock_guard<std::mutex> playstateLock(m_playstate_lock);
            playstateChanged = (m_playstate != PLAYSTATE_PLAY);
            m_playstate = PLAYSTATE_PLAY;
        }
        if (playstateChanged)
            m_playstate_changed.notify_all();
    }

    void Viewer3D::Pause()
    {
        bool playstateChanged = false;
        {
            std::lock_guard<std::mutex> playstateLock(m_playstate_lock);
            playstateChanged = (m_playstate != PLAYSTATE_PAUSE);
            m_playstate = PLAYSTATE_PAUSE;
        }
        if (playstateChanged)
            m_playstate_changed.notify_all();
    }

    void Viewer3D::Step()
    {
        bool playstateChanged = false;
        {
            std::lock_guard<std::mutex> playstateLock(m_playstate_lock);
            playstateChanged = (m_playstate != PLAYSTATE_STEP);
            m_playstate = PLAYSTATE_STEP;
        }
        if (playstateChanged)
            m_playstate_changed.notify_all();
    }

    // Will be synchronously called by the vtk platform, so all
    // calls to vtkRenderer/Window objects are thread safe here
    void Viewer3D::OnPollUpdate(vtkObject* , unsigned long )
    {
        // Update the actors
        {

        }
    }

    bool Viewer3D::AddPoints(
        std::function<bool(
                Eigen::Vector3d &,
                G2D::ViewerColor &)> GetNextPoint)
    {
        {
            // Scoped RAII lock on points
            std::lock_guard<std::mutex> lock(m_points_lock);

            // Allocate colors array
            Eigen::Vector3d pt_eigen;
            G2D::ViewerColor color;
            while (GetNextPoint(pt_eigen, color))
            {
                // Insert new points
                m_pPoints->InsertNextPoint(
                        static_cast<float>(pt_eigen[0]),
                        static_cast<float>(pt_eigen[1]),
                        static_cast<float>(pt_eigen[2]));

                unsigned char color_uch[3] = {
                        color.r,
                        color.g,
                        color.b
                };
                m_pPoints_colors->InsertNextTupleValue(color_uch);
            }

            // Update internal polydata last modified timer
            m_pPoints->Modified();

            // Set the polydata/device mappers to the new data
            m_pPoints_polydata->SetPoints(m_pPoints);

            PolydataAlgoSetInputData(m_pPoints_vertexFilter, m_pPoints_polydata);
            m_pPoints_vertexFilter->Update();

            m_pPoints_filteredPolydata->ShallowCopy(m_pPoints_vertexFilter->GetOutput());

            m_pPoints_filteredPolydata->GetPointData()->SetScalars(m_pPoints_colors);

            // TODO: Verify this variable's usefulness
            // Set state variable so render loop can be notified
            if (!m_pointsAdded.load())
            {
                m_pointsAdded.store(true);
            }
        }

        return true;
    }

    bool Viewer3D::AddPoints(
            const std::vector<Eigen::Vector3d> & points,
            const G2D::ViewerColor & color)
    {
        size_t i = 0;
        AddPoints(
             [&](Eigen::Vector3d & pt, G2D::ViewerColor & c) -> bool
             {
                 if (i >= points.size())
                     return false;

                 pt = points[i];
                 c = color;
                 ++i;

                 return true;
             });
    }

    bool Viewer3D::AddFrustum(
            const double origin[3],
            const double topRight[3],
            const double topLeft[3],
            const double bottomLeft[3],
            const double bottomRight[3],
            const double downVector[3])
    {
        {
            // Scoped RAII lock on points
            std::lock_guard<std::mutex> lock(m_frustums_lock);

            // Add a frustum

            // Insert the point locations of the vertices of the pyramid
            ::vtkIdType tR_id = m_pFrustumPoints->InsertNextPoint(topRight);
            ::vtkIdType tL_id = m_pFrustumPoints->InsertNextPoint(topLeft);
            ::vtkIdType bL_id = m_pFrustumPoints->InsertNextPoint(bottomLeft);
            ::vtkIdType bR_id = m_pFrustumPoints->InsertNextPoint(bottomRight);
            ::vtkIdType or_id = m_pFrustumPoints->InsertNextPoint(origin);

            m_pFrustumPoints->Modified();

            vtkSmartPointer<vtkPyramid> pyramid =
                vtkSmartPointer<vtkPyramid>::New();
            // Point the pyramid vertex IDs to the points IDs they correspond to
            pyramid->GetPointIds()->SetId(0,tR_id);
            pyramid->GetPointIds()->SetId(1,tL_id);
            pyramid->GetPointIds()->SetId(2,bL_id);
            pyramid->GetPointIds()->SetId(3,bR_id);
            pyramid->GetPointIds()->SetId(4,or_id);
            m_vPyramids.push_back(pyramid);

            m_arrUnstructuredGrid->SetPoints(m_pFrustumPoints);
            m_arrUnstructuredGrid->InsertNextCell(pyramid->GetCellType(),pyramid->GetPointIds());

            m_arrUnstructuredGrid->Modified();

            // Also add a cone to represent the "down" vector of a frustum
            if (nullptr != downVector)
            {
                //Create a cone
                vtkSmartPointer<vtkConeSource> coneSource =
                  vtkSmartPointer<vtkConeSource>::New();

                coneSource->SetRadius(0.02);
                double height = 0.5;
                coneSource->SetHeight(height);
                coneSource->SetDirection(
                        downVector[0],
                        downVector[1],
                        downVector[2]);
                coneSource->SetCenter(
                        origin[0] + (0.5 * height * downVector[0]),
                        origin[1] + (0.5 * height * downVector[1]),
                        origin[2] + (0.5 * height * downVector[2]));

                coneSource->Update();

                //Create a mapper and actor
                vtkSmartPointer<vtkPolyDataMapper> mapper =
                  vtkSmartPointer<vtkPolyDataMapper>::New();
                mapper->SetInputConnection(coneSource->GetOutputPort());

                vtkSmartPointer<vtkActor> actor =
                  vtkSmartPointer<vtkActor>::New();
                actor->SetMapper(mapper);

                m_pRenderer->AddActor(actor);
            }

            // TODO: Verify this variable's usefulness
            // Set state variable so render loop can be notified
            if (!m_frustumsAdded.load())
            {
                m_frustumsAdded.store(true);
            }
        }

        return true;
    }

    bool Viewer3D::AddFrustum(
            const double R[9],
            const double t[3])
    {

        return this->AddFrustum(
                Eigen::Matrix<double,3,3,Eigen::ColMajor>(R),
                Eigen::Vector3d(t));
    }

    bool Viewer3D::AddFrustum(
            const Eigen::Quaterniond & r,
            const Eigen::Vector3d & t)
    {
        return this->AddFrustum(
                r.toRotationMatrix(),
                t);
    }

    bool Viewer3D::AddFrustum(
            const Eigen::Vector4d & quaternion,
            const Eigen::Vector3d & t)
    {
        Eigen::Quaterniond q(quaternion);
        return this->AddFrustum(q, t);
    }

    bool Viewer3D::AddFrustum(
            const Eigen::Matrix<double,3,3,Eigen::ColMajor> & R,
            const Eigen::Vector3d & t)
    {
        // Apply affine transform, R|t to the frustum first
        // Assume frustum points towards (0,0,1) in a LH coordinate system
        // Assume R is row-major
        std::vector<Eigen::Vector3d> frustum;
        frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        frustum.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
        frustum.push_back(Eigen::Vector3d(-1.0, 1.0, 1.0));
        frustum.push_back(Eigen::Vector3d(-1.0, -1.0, 1.0));
        frustum.push_back(Eigen::Vector3d(1.0, -1.0, 1.0));

        double scaleFrustum = 1.0 / 3.0; // Scales the size of the frustum

        for (size_t i = 0; i < frustum.size(); i++)
        {
            // y = R*x + t
            Eigen::Vector3d x = scaleFrustum * frustum[i];
            Eigen::Vector3d y = (R * x) + t;
            frustum[i] = y;
        }

        // Add a down vector
        Eigen::Vector3d downVector(0.0, -1.0, 0.0);
        downVector = R * downVector;

        return this->AddFrustum(
                frustum[0].data(),
                frustum[1].data(),
                frustum[2].data(),
                frustum[3].data(),
                frustum[4].data(),
                downVector.data());
    }
}
