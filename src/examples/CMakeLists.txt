add_executable(object_detection_app object_detection_app.cpp
)
target_link_libraries(object_detection_app
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

