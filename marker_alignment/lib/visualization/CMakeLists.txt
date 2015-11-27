# For VTK to help visualize stuff
include_directories(
  ${PROJECT_SOURCE_DIR}/marker_alignment/lib
)

ADD_LIBRARY(viewer3d_library SHARED
 Viewer3D.h
 Viewer3D.cpp
 ViewerButton.h
 ViewerColors.h
)

# SET_TARGET_PROPERTIES(viewer3d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}viewer3d)

TARGET_LINK_LIBRARIES(viewer3d_library
  ${OpenCV_LIBS} 
  ${VTK_LIBRARIES}
)
