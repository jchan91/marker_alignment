#pragma once

#include "ViewerColors.h"

#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <Eigen/Core>

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkConeSource.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>

namespace G2D
{
    class Viewer3D
    {
    public:
        Viewer3D();
        ~Viewer3D();

        void InitializeAndRun();
        void InitializeAndRunAsync();

        // Description:
        // When a user makes this call, their thread will go to sleep
        // if the viewer has been asked to "pause" by the user, allowing
        // the user to analyze the rendered data's current state. If the
        // user has pressed "play", then MaybeYieldToViewer will immediately
        // return.
        void MaybeYieldToViewer();

        void Play();
        void Pause();
        void Step();

        // TODO: Do we really need this?
        struct ViewerAddOpts
        {
            bool NoCopy = true; // True: Dereference that pointer only, don't make copy of data
            bool Blocking = false; // True: Block until points added // TODO: Implement this feature
        };

        // Description:
        // Adds 3D points to be rendered in the paritcular color.
        // GetNextPoint is a user defined callback to retrieve the next point and its associated color
        bool AddPoints(
                std::function<bool(Eigen::Vector3d &, G2D::ViewerColor &)> GetNextPoint,
                const ViewerAddOpts* pOpts = nullptr);

    private:
        // Description:
        // Playstate represents the state of the user of the viewer,
        // not so much the viewer itself. For instance, Pause is designed
        // to be initiated by the user to pause their algorithm, and render
        // current data. Step would be allowing their algorithm to continue
        // until the next MaybeYieldToViewer().
        enum Playstate
        {
            PLAYSTATE_PLAY = 0,
            PLAYSTATE_STEP,
            PLAYSTATE_PAUSE,
        };

        // Description:
        // A timer event periodically calls this function asking if any updates
        // have been made to the viewer (e.g. AddPoints called?)
        void OnPollUpdate(vtkObject* caller, unsigned long eventId);

        // VTK Renderer/Windows
        ::vtkRenderer* m_pRenderer;
        ::vtkRenderWindow* m_pRenderWindow;
        ::vtkRenderWindowInteractor* m_pRenderWindowInteractor;
        std::thread m_renderThread;

        // Playstate
        std::mutex m_playstate_lock;
        std::condition_variable m_playstate_changed;
        Playstate m_playstate;

        // Data to render. Must acquire lock before using any of the data being
        // rendered.

        // Points
        std::atomic_bool m_pointsAdded;
        std::mutex m_points_lock;
        ::vtkSmartPointer<::vtkPoints> m_pPoints;
        ::vtkSmartPointer<::vtkCellArray> m_pPoints_vertices;
        ::vtkSmartPointer<::vtkPolyData> m_pPoints_polydata;
        ::vtkSmartPointer<::vtkActor> m_pPoints_actor;

        // TODO: Remove test code
        ::vtkConeSource* m_pConeSrc;
    };
}
