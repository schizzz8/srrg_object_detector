#pragma once

#include <set>

#include <srrg_types/cloud_3d.h>
#include "simple_viewer.h"

#include <srrg_object_detector/object_detector.h>

class DetectorViewer : public SimpleViewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DetectorViewer();
  enum Mode {MoveCamera=0x0, MoveObject=0x1};

  virtual void draw();
  virtual void keyPressEvent(QKeyEvent *e);

  inline void setDetector(ObjectDetector* detector_){
    _detector = detector_;
    _cloud = _detector->depthCloud();
    _bounding_boxes = _detector->boundingBoxes();
  }

protected:
  Mode _mode;
  bool _is_orthographic;

  ObjectDetector* _detector;

  srrg_core::Cloud3D _cloud;
  ObjectDetector::BoundingBoxes3D _bounding_boxes;


};
