#pragma once

#include "ViewerColors.h"

#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkConeSource.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>

#include <vtkPyramid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>

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

        // Description:
        // Adds 3D points to be rendered in the paritcular color.
        // GetNextPoint is a user defined callback to retrieve the next point and its associated color
        bool AddPoints(std::function<bool(Eigen::Vector3d &, G2D::ViewerColor &)> GetNextPoint);

        // Description:
        // Adds a pyramid to the scene. Useful for representing a camera pose
        // @ret Returns the ID of a frustum, which can be used later to identify it
        bool AddFrustum(
                const double origin[3],
                const double topRight[3],
                const double topLeft[3],
                const double bottomLeft[3],
                const double bottomRight[3]);

        // Description:
        // Given an origin, and the affine transform representing the pose of the frustum centered at
        // origin. Assumes LH, and frustum direction is [0,0,1], and gets transformed from world2frustum
        // by R|t.
        // @param R is a column-major rotation matrix.
        bool AddFrustum(
                const double R[9],
                const double t[3]);

        bool AddFrustum(
                const Eigen::Quaterniond & r,
                const Eigen::Vector3d & t);

        bool AddFrustum(
                const Eigen::Matrix<double,3,3,Eigen::ColMajor> & R,
                const Eigen::Vector3d & t);

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

        // Decription:
        // VTK helpers to deal with legacy vtk versions
        static void PolydataAlgoSetInputData(::vtkPolyDataAlgorithm* pPolydata, ::vtkDataObject* pInput);
        static void PolydataMapperSetPolyData(::vtkPolyDataMapper* pMapper, ::vtkPolyData* pPolyData);
        static void DataSetMapperSetDataSet(::vtkDataSetMapper* pMapper, ::vtkDataSet* pDataSet);

        // Description:
        // Helper initializer meant for the Viewer ctor. Sets up the associated vtk variables
        // to display colored points
        void SetupVtkColoredPoints();

        // Description:
        // Helper initializer meant for the Viewer ctor. Sets up the associated vtk variables
        // to display pyramids that represent the camera pose, and FOV (estimate)
        void SetupVtkPoseFrustums();

        // Description:
        // Helper to create an xyz axis to help user comprehend orientation
        void SetupAxes();

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

        // Colored Points
        std::atomic_bool m_pointsAdded;
        std::mutex m_points_lock;
        ::vtkSmartPointer<::vtkPoints> m_pPoints;
        ::vtkSmartPointer<::vtkVertexGlyphFilter> m_pPoints_vertexFilter;
        ::vtkSmartPointer<::vtkPolyData> m_pPoints_polydata;
        ::vtkSmartPointer<::vtkPolyData> m_pPoints_filteredPolydata;
        ::vtkSmartPointer<::vtkUnsignedCharArray> m_pPoints_colors;
        ::vtkSmartPointer<::vtkActor> m_pPoints_actor;

        // Pose Frustums (represented as pyramids)
        // Pyramid "width/length" determined by camera intrinsics (focal lengths)
        std::atomic_bool m_frustumsAdded;
        std::mutex m_frustums_lock;
        ::vtkSmartPointer<::vtkPoints> m_pFrustumPoints;
        std::vector<::vtkSmartPointer<::vtkPyramid>> m_vPyramids;
        ::vtkSmartPointer<::vtkUnstructuredGrid> m_arrUnstructuredGrid;
        ::vtkSmartPointer<::vtkActor> m_pFrustums_actor;

        // TODO: Remove test code
        ::vtkConeSource* m_pConeSrc;
    };
}
