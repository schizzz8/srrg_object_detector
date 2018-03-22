#include <iostream>
#include <fstream>
#include <limits>
#include <deque>
#include <queue>
#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <qapplication.h>
#include <qglviewer.h>

#include <srrg_system_utils/system_utils.h>
#include <srrg_object_detector_viewers/detector_viewer.h>

using namespace std;
using namespace Eigen;
using namespace srrg_core;

int main(int argc, char** argv) {

  //read images
  cv::Mat rgb_image = cv::imread(argv[1],cv::IMREAD_UNCHANGED);
  cv::Mat raw_depth_image = cv::imread(argv[2],cv::IMREAD_UNCHANGED);

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
          0.0, 554.25, 240.5,
          0.0,    0.0,   1.0;

  //log
  char* filename = argv[3];

  ObjectDetector detector;
  detector.setK(K);
  detector.setImages(rgb_image,raw_depth_image);
  detector.readData(filename);
  detector.compute();

//  cv::Mat label_image = detector.labelImage().clone();
//  cv::imshow("label_image",label_image);
//  cv::waitKey();

  QApplication app(argc, argv);
  DetectorViewer viewer;
  viewer.setDetector(&detector);
  viewer.show();
  app.exec();


  return 0;
}
