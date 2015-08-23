#include "Viewer3D.h"

// TODO: Remove conesource
#include <vtkConeSource.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <functional>
#include <thread>

namespace G2D
{
    class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
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

        // Create the possible actors
        m_pPoints = ::vtkSmartPointer<::vtkPoints>(::vtkPoints::New());
        m_pPoints_vertices = ::vtkSmartPointer<::vtkCellArray>(::vtkCellArray::New());

        m_pPoints_polydata = ::vtkSmartPointer<::vtkPolyData>(::vtkPolyData::New());
        m_pPoints_polydata->SetPoints(m_pPoints.GetPointer());
        m_pPoints_polydata->SetVerts(m_pPoints_vertices.GetPointer());

        auto pPointsMapper = ::vtkSmartPointer<::vtkPolyDataMapper>(::vtkPolyDataMapper::New());
        #if VTK_MAJOR_VERSION <= 5
        pPointsMapper->SetInput(m_pPoints_polydata);
        #else
        pPointsMapper->SetInputData(m_pPoints_polydata);
        #endif

        m_pPoints_actor = ::vtkSmartPointer<::vtkActor>(::vtkActor::New());
        m_pPoints_actor->SetMapper(pPointsMapper);
        m_pPoints_actor->GetProperty()->SetPointSize(20); // Default point size 20
        m_pRenderer->AddActor(m_pPoints_actor);

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
        m_pRenderWindowInteractor->AddObserver(::vtkCommand::TimerEvent, pCallback);
        m_pRenderWindowInteractor->CreateRepeatingTimer(100); // Fire every 100 ms

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
        std::function<bool(Eigen::Vector3d &, G2D::ViewerColor &)> GetNextPoint,
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
                Eigen::Vector3d pt_eigen;
                G2D::ViewerColor color;
                while (GetNextPoint(pt_eigen, color))
                {
                    float pt_f[3] = {
                            static_cast<float>(pt_eigen[0]),
                            static_cast<float>(pt_eigen[1]),
                            static_cast<float>(pt_eigen[2])
                    };
                    ::vtkIdType pid[1];
                    pid[0] = this->m_pPoints->InsertNextPoint(pt_f[0], pt_f[1], pt_f[2]);
                    this->m_pPoints_vertices->InsertNextCell(1, pid);

                    // TODO: Should this be moved to OnPollUpdate, in case of race condition with render thread?
                    this->m_pPoints->Modified(); // Tell the renderer points have been modified
                    // TODO: Incorporate color

                    if (!m_pointsAdded.load())
                    {
                        m_pointsAdded.store(true);
                    }
                }
            }
            else
            {
                throw new std::exception();
            }
        }


        return true;
    }
}
