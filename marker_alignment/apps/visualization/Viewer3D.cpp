#include "Viewer3D.h"

#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <vtkConeSource.h>

#include <iostream>
#include <functional>
#include <thread>

namespace G2D
{
    bool Viewer3D::CreateAndInit(Viewer3D** ppViewer)
    {
        Viewer3D* pViewer = new Viewer3D;
//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//        pcl::visualization::CloudViewer viewer("Cloud Viewer");
        pViewer->m_viewer = new pcl::visualization::CloudViewer("Cloud Viewer");

//        pViewer->m_newCloudDataReady.load(false);
//        pViewer->m_cloudListBusy.load(false);

        *ppViewer = pViewer;
        return true;
    }

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
            std::cout << "TimerCount: " << this->TimerCount << std::endl;
        }

    private:
        // TODO: Remove this
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
    }

    void RunViewerAsyncCallback(Viewer3D* pViewer)
    {
        // Render/Interact
        pViewer->InitializeAndRun();
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
                [&](vtkObject* caller, unsigned long eventId)
                {
                    this->OnPollUpdate(caller, eventId);
                };
        m_pRenderWindowInteractor->AddObserver(vtkCommand::TimerEvent, pCallback);
        m_pRenderWindowInteractor->CreateRepeatingTimer(100); // Fire every 100 ms

        // Start Render/Interact
        m_playstate = PLAYSTATE_PAUSE; // Pause and give user a chance for control
        m_pRenderWindow->Render();
        m_pRenderWindowInteractor->Initialize();
        m_pRenderWindowInteractor->Start();
    }

    void Viewer3D::Run()
    {
//        RunViewer(m_pRenderWindow, m_pRenderWindowInteractor);
    }

    void Viewer3D::RunAsync()
    {
//        m_viewer->showCloud(m_pCloud);

//        boost::function1<void, pcl::visualization::PCLVisualizer&> itr = this->DoItr;
//        std::function<void(pcl::visualization::PCLVisualizer&)> itr = this->DoItr;
//        m_viewer->runOnVisualizationThread(itr);

        // Start VTK WindowRenderInteractor on another thread
        std::thread renderThread(RunViewerAsyncCallback, this);
    }

    bool Viewer3D::AddPoints(const std::vector<Eigen::Vector3d>* pPoints)
    {
//        for (Eigen::Vector3d pt : *pPoints)
//        {
//            pcl::PointXYZRGB pt_pcl(pt[0], pt[1], pt[2]);
//            m_pCloud->push_back(pt_pcl);
//        }


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
        std::unique_lock<std::mutex>(m_playstate_lock);
        m_playstate_changed.wait(m_playstate_lock, [this]{ return m_playstate != PLAYSTATE_PAUSE; });

        // Free the lock and let caller resume work
        m_playstate_lock.unlock();
    }

    void Viewer3D::Play()
    {
        std::lock_guard<std::mutex> playstateLock(m_playstate_lock);
        m_playstate = PLAYSTATE_PLAY;
        m_playstate_changed.notify_all();
    }

    void Viewer3D::Pause()
    {
        std::lock_guard<std::mutex> playstateLock(m_playstate_lock);
        m_playstate = PLAYSTATE_PAUSE;
        m_playstate_changed.notify_all();
    }

    void Viewer3D::Step()
    {
        std::lock_guard<std::mutex> playstateLock(m_playstate_lock);
        m_playstate = PLAYSTATE_STEP;
        m_playstate_changed.notify_all();
    }

    // Will be synchronously called by the vtk platform, so all
    // calls to vtkRenderer/Window objects are thread safe here
    void Viewer3D::OnPollUpdate(vtkObject* , unsigned long )
    {
        // Update the actors
        this->AddPoints(nullptr); // TODO: Remove this

        // If user has only asked to step, then set playstate
        // back to a paused
        {
            std::lock_guard<std::mutex> playstateLock(m_playstate_lock); // RAII Scoped lock
            if (m_playstate == PLAYSTATE_STEP)
            {
                m_playstate = PLAYSTATE_PAUSE;
            }
        }
    }
}
