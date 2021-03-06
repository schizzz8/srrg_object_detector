#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

class Detection;

typedef std::shared_ptr<Detection> DetectionPtr;
typedef std::vector<DetectionPtr> Detections;

class Detection{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Detection(const std::string& type_="",
            const Eigen::Vector2i& top_left_ = Eigen::Vector2i(10000,10000),
            const Eigen::Vector2i& bottom_right_ = Eigen::Vector2i(-10000,-10000),
            const std::vector<Eigen::Vector2i>& pixels_ = std::vector<Eigen::Vector2i>(640*480)):
    _type(type_),
    _top_left(top_left_),
    _bottom_right(bottom_right_),
    _pixels(pixels_){}

  inline const std::string& type() const {return _type;}
  inline std::string& type() {return _type;}
  inline const Eigen::Vector2i& topLeft() const {return _top_left;}
  inline Eigen::Vector2i& topLeft() {return _top_left;}
  inline const Eigen::Vector2i& bottomRight() const {return _bottom_right;}
  inline Eigen::Vector2i& bottomRight() {return _bottom_right;}
  inline const std::vector<Eigen::Vector2i>& pixels() const {return _pixels;}
  inline std::vector<Eigen::Vector2i>& pixels() {return _pixels;}
  inline const int size() const {return _size;}
  inline int &size() {return _size;}

private:
  std::string _type;
  Eigen::Vector2i _top_left;
  Eigen::Vector2i _bottom_right;
  std::vector<Eigen::Vector2i> _pixels;
  int _size = 0;
};

class ObjectDetector{
public:
  struct Model{
    Model(std::string type_ = "",
          Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity(),
          Eigen::Vector3f min_ = Eigen::Vector3f::Zero(),
          Eigen::Vector3f max_ = Eigen::Vector3f::Zero()):
      _type(type_),
      _pose(pose_),
      _min(min_),
      _max(max_){}
    std::string _type;
    Eigen::Isometry3f _pose;
    Eigen::Vector3f _min;
    Eigen::Vector3f _max;
  };

  typedef std::pair<Eigen::Vector3f,Eigen::Vector3f> BoundingBox3D;
  typedef std::vector<BoundingBox3D> BoundingBoxes3D;
  typedef std::vector<Model> Models;

  ObjectDetector(){}

  inline void setK(const Eigen::Matrix3f& K_){_K = K_;}

  void setImages(const srrg_core::RGBImage& rgb_image_,
                 const srrg_core::RawDepthImage& raw_depth_image_);

  void readData(char* filename);

  void compute();

  inline const Eigen::Isometry3f& rgbdCameraTransform() const {return _rgbd_camera_transform;}
  inline const Eigen::Isometry3f& logicalCameraTransform() const {return _logical_camera_transform;}
  inline const srrg_core::RGBImage& labelImage() const {return _label_image;}
  inline const srrg_core::Cloud3D& depthCloud() const {return _depth_cloud;}
  inline const BoundingBoxes3D& boundingBoxes() const {return _bounding_boxes;}

protected:
  cv::Mat _rgb_image;
  cv::Mat _raw_depth_image;
  int _rows;
  int _cols;
  Eigen::Matrix3f _K;
  srrg_core::Float3Image _points_image;
  srrg_core::Cloud3D _depth_cloud;

  Eigen::Isometry3f _rgbd_camera_transform;
  Eigen::Isometry3f _logical_camera_transform;
  std::vector<Model> _models;

  BoundingBoxes3D _bounding_boxes;
  Detections _detections;

  srrg_core::RGBImage _label_image;

private:
  void computeWorldBoundingBoxes();

  inline bool inRange(const Eigen::Vector3f &point, const BoundingBox3D &bounding_box){
    return (point.x() >= bounding_box.first.x()-0.01 && point.x() <= bounding_box.second.x()+0.01 &&
            point.y() >= bounding_box.first.y()-0.01 && point.y() <= bounding_box.second.y()+0.01 &&
            point.z() >= bounding_box.first.z()-0.01 && point.z() <= bounding_box.second.z()+0.01);
  }

  void computeImageBoundingBoxes();

  cv::Vec3b type2color(std::string type);

  void computeLabelImage();

};
