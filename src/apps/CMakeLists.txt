add_executable(object_detector_app object_detector_app.cpp)
target_link_libraries(object_detector_app
  srrg_object_detector_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
