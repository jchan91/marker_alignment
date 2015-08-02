#pragma once

#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <Eigen/Core>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkConeSource.h>

namespace G2D
{
    class Viewer3D
    {
    public:
        static bool CreateAndInit(Viewer3D** ppViewer);

        Viewer3D();
        ~Viewer3D();

        void InitializeAndRun();
        void InitializeAndRunAsync();

        void Run();

        void RunAsync();

        void MaybeYieldToViewer();

        void Play();
        void Pause();
        void Step();

        bool AddPoints(const std::vector<Eigen::Vector3d>* pPoints);

    private:
        enum Playstate
        {
            PLAYSTATE_PLAY = 0,
            PLAYSTATE_STEP,
            PLAYSTATE_PAUSE,
        };

        void DoItr(pcl::visualization::PCLVisualizer& viewer);

        void OnPollUpdate(vtkObject* caller, unsigned long eventId);

        // PCL Member stuff
        pcl::visualization::CloudViewer* m_viewer;

        // VTK Renderer/Windows
        ::vtkRenderer* m_pRenderer;
        ::vtkRenderWindow* m_pRenderWindow;
        ::vtkRenderWindowInteractor* m_pRenderWindowInteractor;


        std::mutex m_playstate_lock;
        std::condition_variable m_playstate_changed;
        Playstate m_playstate;

        // TODO: Remove test code
        ::vtkConeSource* m_pConeSrc;
//        std::atomic<bool> m_newCloudDataReady;
//        std::atomic<bool> m_cloudListBusy;
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pCloud;

    };
}
