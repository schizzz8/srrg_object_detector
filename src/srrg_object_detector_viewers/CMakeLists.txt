add_library(srrg_object_detector_viewers_library SHARED
  detector_viewer.cpp detector_viewer.h
  simple_viewer.cpp simple_viewer.h
)

target_link_libraries(srrg_object_detector_viewers_library
  srrg_object_detector_library
  ${catkin_LIBRARIES}
  ${QGLVIEWER_LIBRARY} 
  ${SRRG_QT_LIBRARIES} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
