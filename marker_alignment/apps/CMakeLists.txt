#INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

#ADD_LIBRARY(marker_alignment_library ${MARKER_ALIGNMENT_LIB_TYPE}
#  CameraIntrinsics.h
#  CameraState.h
#  MockData.h
#  Pose.h
#)

#SET_TARGET_PROPERTIES(marker_alignment_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}marker_alignment)

INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

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
  main.cpp
)

SET_TARGET_PROPERTIES(marker_alignment PROPERTIES OUTPUT_NAME marker_alignment${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(marker_alignment ${CERES_LIBRARIES})
