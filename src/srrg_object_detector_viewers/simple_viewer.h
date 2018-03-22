#pragma once

#include <QKeyEvent>
#include <QGLViewer/qglviewer.h>

class SimpleViewer: public QGLViewer {
public:

  //! ctor
  SimpleViewer(QWidget* parent = 0);

  //! init method, opens the gl viewport and sets the key bindings
  void init();

  //! callback invoked by the application on new key event. It saves the last event in
  //! a member variable
  virtual void keyPressEvent(QKeyEvent *e);

  //! returns the last key pressed since invoking keyEventProcessed();
  QKeyEvent* lastKeyEvent();

  //! call this to clear the events, after processing them
  void keyEventProcessed();

protected:

  QKeyEvent _last_key_event;
  bool _last_key_event_processed;

};
