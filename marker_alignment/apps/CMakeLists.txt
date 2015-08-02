#INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

#ADD_LIBRARY(marker_alignment_library ${MARKER_ALIGNMENT_LIB_TYPE}
#  CameraIntrinsics.h
#  CameraState.h
#  MockData.h
#  Pose.h
#)

#SET_TARGET_PROPERTIES(marker_alignment_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}marker_alignment)

INCLUDE_DIRECTORIES(
  ${CERES_INCLUDE_DIRS}
  visualization
)

ADD_EXECUTABLE(marker_alignment
  CameraIntrinsics.h
  CameraState.h
  MockData.h
  Pose.h
  se3quat.h
  se3_ops.h
  se3_ops.hpp
  math_constants.h
  MarkerAlignmentProblem.h
  visualization/Viewer3D.h
  visualization/Viewer3D.cpp
  visualization/ViewerColors.h
  visualization/ViewerButton.h
  main.cpp
)

SET_TARGET_PROPERTIES(marker_alignment PROPERTIES OUTPUT_NAME marker_alignment${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(marker_alignment 
  ${CERES_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${Boost_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES} 
  ${PCL_IO_LIBRARIES}
  ${VTK_LIBRARIES}
)
