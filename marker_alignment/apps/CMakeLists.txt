include_directories(
  ${PROJECT_SOURCE_DIR}/marker_alignment/lib
)

ADD_EXECUTABLE(marker_alignment
  CameraIntrinsics.h
  CameraState.h
  MockData.h
  Pose.h
  # se3quat.h
  # se3_ops.h
  # se3_ops.hpp
  # math_constants.h
  MarkerAlignmentProblem.h
  main.cpp
)

SET_TARGET_PROPERTIES(marker_alignment PROPERTIES OUTPUT_NAME marker_alignment${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(marker_alignment 
  viewer3d_library
  ${CERES_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${Boost_LIBRARIES} 
)
