#include <cstring>

#include <srrg_gl_helpers/opengl_primitives.h>
#include <srrg_gl_helpers/draw_attributes.h>

#include "detector_viewer.h"

using namespace std;
using namespace Eigen;
using namespace srrg_gl_helpers;
using namespace srrg_core;

DetectorViewer::DetectorViewer() {
  _mode = MoveCamera;
  _is_orthographic = false;
}


void DetectorViewer::draw(){
  _cloud.draw();
  for(int i=0; i<_bounding_boxes.size(); ++i){
    const Eigen::Vector3f& min=_bounding_boxes[i].first;
    const Eigen::Vector3f& max=_bounding_boxes[i].second;
    const Eigen::Vector3f centroid = (min + max)/2.0f;
    const Eigen::Vector3f half_size= (max - min)/2.0f;

    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.translation() = centroid;

    glPushMatrix();
    glMultMatrix(transform);
    drawBoxWireframe(half_size.x(),half_size.y(),half_size.z());
    glPopMatrix();

  }
}

void DetectorViewer::keyPressEvent(QKeyEvent *e) {
  switch(e->key()){
    case Qt::Key_Space:
      _mode = MoveCamera;
      cerr << "MoveCamera Mode is OFF" << endl;
      return;
    case Qt::Key_M:
      cerr << "MoveObject Mode is ON" << endl;
      _mode = MoveObject;
      return;
    case Qt::Key_O:
      _is_orthographic =  !_is_orthographic;
      if(_is_orthographic)
        camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);
      else
        camera()->setType(qglviewer::Camera::PERSPECTIVE);
      updateGL();
      return;
    default:;
  }
  if (_mode != MoveObject){
    SimpleViewer::keyPressEvent(e);
    return;
  }

  updateGL();
}
