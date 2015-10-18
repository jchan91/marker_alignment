#include "Viewer3D.h"

// TODO: Remove conesource
#include <vtkConeSource.h>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCommand.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkPointData.h>

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
        // TODO: Implement something to kill the thread?
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

    void Viewer3D::InitializeAndRun()
    {
        // Create render/window/interactor
        // TODO: Convert these to smart pointers
        m_pRenderer = ::vtkRenderer::New();
        m_pRenderWindow = ::vtkRenderWindow::New();
        m_pRenderWindow->AddRenderer(m_pRenderer);
        m_pRenderWindowInteractor = ::vtkRenderWindowInteractor::New();
        m_pRenderWindowInteractor->SetRenderWindow(m_pRenderWindow);
        m_pRenderWindowInteractor->Initialize();

        // Create the "actors" (e.g. point clouds, pose frustums, etc.)

        // Add points
        SetupVtkColoredPoints();

        // TODO: Remove this test code
        // Create a cone
//        m_pConeSrc = ::vtkConeSource::New();
//        m_pConeSrc->SetResolution(8);
//        ::vtkPolyDataMapper* pMapper = ::vtkPolyDataMapper::New();
//        pMapper->SetInput(m_pConeSrc->GetOutput());
//        ::vtkActor* pActor = ::vtkActor::New();
//        pActor->SetMapper(pMapper);
//        // Set the actor into the scene
//        m_pRenderer->AddActor(pActor);

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
                G2D::ViewerColor &)> GetNextPoint,
        const ViewerAddOpts* pOpts)
    {
        Viewer3D::ViewerAddOpts options; // Use initialized defaults
        if (nullptr != pOpts)
        {
            options = *pOpts;
        }

        {
            // Scoped RAII lock on points
            std::lock_guard<std::mutex> lock(m_points_lock);

            if (options.NoCopy)
            {
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
            }
            else
            {
                throw new std::exception();
            }
        }


        return true;
    }
}
