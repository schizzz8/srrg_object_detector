#add_executable(object_detector_app object_detector_app.cpp)
#target_link_libraries(object_detector_app
#  srrg_object_detector_library
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)

add_executable(object_detector_gui_app object_detector_gui_app.cpp)
target_link_libraries(object_detector_gui_app
  srrg_object_detector_viewers_library
  srrg_object_detector_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
