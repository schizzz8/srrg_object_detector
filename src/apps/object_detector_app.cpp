#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <srrg_object_detector/object_detector.h>

int main(int argc, char** argv){

  //read images
  cv::Mat rgb_image = cv::imread(argv[1],cv::IMREAD_UNCHANGED);
  cv::Mat raw_depth_image = cv::imread(argv[2],cv::IMREAD_UNCHANGED);

  //K
  Eigen::Matrix3f K;
  K << 277.127, 0.0, 160.5, 0.0, 277.127, 120.5, 0.0, 0.0, 1.0;

  //log
  char* filename = argv[3];

  ObjectDetector detector;
  detector.setK(K);
  detector.setImages(rgb_image,raw_depth_image);
  detector.readData(filename);
  detector.compute();

  cv::Mat label_image = detector.labelImage().clone();

  cv::imshow("label_image",label_image);
  cv::waitKey();

  return 0;
}
