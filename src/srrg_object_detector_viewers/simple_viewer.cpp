#include "simple_viewer.h"
#include <cstring>

// Macro helpers for identifying the version number of QGLViewer
// QGLViewer changed some parts of its API in version 2.6.
// The following preprocessor hack accounts for this.
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 6)
#define qglv_real qreal
#else
#define qglv_real float
#endif

using namespace std;

class StandardCamera: public qglviewer::Camera {
public:
  StandardCamera(): _standard(true) {}

  qglv_real zNear() const {
    if(_standard) { return qglv_real(0.001f); }
    else { return Camera::zNear(); }
  }

  qglv_real zFar() const {
    if(_standard) { return qglv_real(10000.0f); }
    else { return Camera::zFar(); }
  }

  bool standard() const { return _standard; }
  void setStandard(bool s) { _standard = s; }

protected:
  bool _standard;
};

SimpleViewer::SimpleViewer(QWidget* parent): QGLViewer(parent),
  _last_key_event(QEvent::None, 0, Qt::NoModifier),
  _last_key_event_processed(true) {}

void SimpleViewer::init() {
  // Init QGLViewer.
  QGLViewer::init();
  // Set background color light yellow.
  // setBackgroundColor(QColor::fromRgb(255, 255, 194));


  // Set background color white.
//  glClearColor(1.0f,1.0f,1.0f,1.0f);
  //setBackgroundColor(QColor::fromRgb(255, 255, 255));

  QColor white = QColor(Qt::white);
  setBackgroundColor(white);


  const GLfloat pos[4] = {0.f, -1.f, -1.f, 1.0f};
  glLightfv(GL_LIGHT1, GL_POSITION, pos);
  glDisable(GL_LIGHT0);
  glEnable(GL_LIGHT1);

  const GLfloat light_ambient[4] = {.5f,.5f,.5f, .5f};
  glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);

  const GLfloat light_diffuse[4] = {1.0, 1.0, 1.0, 1.0};
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);

  // Set some default settings.
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glShadeModel(GL_FLAT);
 glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Don't save state.
  setStateFileName(QString::null);

  // Mouse bindings.
  setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
  setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);

  // Replace camera.
  qglviewer::Camera *oldcam = camera();
  qglviewer::Camera *cam = new StandardCamera();
  setCamera(cam);
  //  cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 10.0f));
  //  cam->setUpVector(qglviewer::Vec(1.0f, 0.0f, 0.0f));
  //  cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));

  srrg_core::Vector6f v;
  v << 0.0f,0.0f,0.0f,-0.5f,0.5f,-0.5f;
  Eigen::Isometry3f t = srrg_core::v2t(v);
  Eigen::Vector3f p (0,0,-2);
  Eigen::Vector3f d = t.rotation()*Eigen::Vector3f(0,-1,0);
  cam->setPosition(qglviewer::Vec(p.x(),p.y(),p.z()));
  cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
  cam->lookAt(qglviewer::Vec(p.x()+d.x(),p.y()+d.y(),p.z()+d.z()));

  delete oldcam;

}



void SimpleViewer::keyPressEvent(QKeyEvent *e) {
  QGLViewer::keyPressEvent(e);
  _last_key_event = *e;
  _last_key_event_processed=false;
}

QKeyEvent* SimpleViewer::lastKeyEvent() {
  if (_last_key_event_processed)
    return 0;
  return &_last_key_event;
}

void SimpleViewer::keyEventProcessed() {
  _last_key_event_processed = true;
}
