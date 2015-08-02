#include "Viewer3D.h"

// TODO: Remove conesource
#include <vtkConeSource.h>


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

        // TODO: Remove this test code
        // Create a cone
        m_pConeSrc = ::vtkConeSource::New();
        m_pConeSrc->SetResolution(8);
        ::vtkPolyDataMapper* pMapper = ::vtkPolyDataMapper::New();
        pMapper->SetInput(m_pConeSrc->GetOutput());
        ::vtkActor* pActor = ::vtkActor::New();
        pActor->SetMapper(pMapper);
        // Set the actor into the scene
        m_pRenderer->AddActor(pActor);

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

    bool Viewer3D::AddPoints(const std::vector<Eigen::Vector3d>* pPoints)
    {
        // TODO: Maybe this function shouldn't directly update actors
        // Maybe just queue up request here and let poll do the updating
        double xyz[3];
        m_pConeSrc->GetCenter(xyz);
        m_pConeSrc->SetCenter(xyz[0] + 0.1, xyz[1] + 0.1, xyz[2]);
        m_pRenderWindow->Render();

        return true;
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
        this->AddPoints(nullptr); // TODO: Remove this
    }
}
