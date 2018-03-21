#include "object_detector.h"

using namespace srrg_core;

void ObjectDetector::setImages(const srrg_core::RGBImage &rgb_image_,
                               const srrg_core::RawDepthImage &raw_depth_image_){
  //copy images
  _rgb_image = rgb_image_;
  _raw_depth_image = raw_depth_image_;

  _rows = _rgb_image.rows;
  _cols = _rgb_image.cols;

  //compute points image
  srrg_core::FloatImage depth_image;
  srrg_core::Float3Image directions_image;

  directions_image.create(_rows,_cols);
  initializePinholeDirections(directions_image,_K);
  _points_image.create(_rows,_cols);
  convert_16UC1_to_32FC1(depth_image, _raw_depth_image);
  computePointsImage(_points_image,
                     directions_image,
                     depth_image,
                     0.04f,
                     8.0f);

  _label_image.create(_rows,_cols);
  _label_image=0;
}

void ObjectDetector::readData(char *filename){

  std::string line;
  std::ifstream data(filename);

  if(data.is_open()) {
    if(std::getline(data,line)) {
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      _rgbd_camera_transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      _rgbd_camera_transform.linear().matrix() = R;
    }
    if(std::getline(data,line)) {
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      _logical_camera_transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      _logical_camera_transform.linear().matrix() = R;
    }
    int n;
    if(std::getline(data,line)) {
      std::istringstream iss(line);
      iss >> n;
    }
    for(int i=0; i<n; i++){
      std::getline(data,line);
      std::istringstream iss(line);
      std::string type;
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      double minx,miny,minz,maxx,maxy,maxz;
      iss >> type;
      Eigen::Isometry3f model_pose=Eigen::Isometry3f::Identity();
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      model_pose.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      model_pose.linear().matrix() = R;
      iss >> minx>>miny>>minz>>maxx>>maxy>>maxz;
      Eigen::Vector3f min(minx,miny,minz);
      Eigen::Vector3f max(maxx,maxy,maxz);

      Model model(type,model_pose,min,max);
      _models.push_back(model);
    }
    data.close();
  }

  std::cerr << "RGBD camera pose" << std::endl;
  std::cerr << "position:" << std::endl;
  std::cerr << _rgbd_camera_transform.translation().transpose() << std::endl;
  std::cerr << "orientation:" << std::endl;
  std::cerr << _rgbd_camera_transform.linear().matrix() << std::endl << std::endl;

  std::cerr << "Logical camera pose" << std::endl;
  std::cerr << "position:" << std::endl;
  std::cerr << _logical_camera_transform.translation().transpose() << std::endl;
  std::cerr << "orientation:" << std::endl;
  std::cerr << _logical_camera_transform.linear().matrix() << std::endl << std::endl;

  int num_models=_models.size();
  std::cerr << "Detected " << num_models << " models" << std::endl;
  for(int i=0; i<num_models; ++i){
    const Model &model = _models[i];
    std::cerr << model._type << std::endl;
    const Eigen::Isometry3f model_transform=model._pose;
    std::cerr << "position:" << std::endl;
    std::cerr << model_transform.translation().transpose() << std::endl;
    std::cerr << "orientation:" << std::endl;
    std::cerr << model_transform.linear().matrix() << std::endl;
    std::cerr << "Min:" << std::endl;
    std::cerr << model._min.transpose() << std::endl;
    std::cerr << "Max:" << std::endl;
    std::cerr << model._max.transpose() << std::endl << std::endl;
  }
}

void ObjectDetector::computeWorldBoundingBoxes(){
  Eigen::Isometry3f transform = _rgbd_camera_transform.inverse()*_logical_camera_transform;
  int num_models=_models.size();
  _bounding_boxes.resize(num_models);
  _detections.resize(num_models);

  for(int i=0; i<num_models; ++i){
    const Model &model = _models[i];
    const Eigen::Isometry3f& model_pose=model._pose;

    std::vector<Eigen::Vector3f> points;
    points.push_back(transform*model_pose*Eigen::Vector3f(model._min.x(),model._min.y(),model._min.z()));
    points.push_back(transform*model_pose*Eigen::Vector3f(model._max.x(),model._max.y(),model._max.z()));

    float x_min=100000,x_max=-100000,y_min=100000,y_max=-100000,z_min=100000,z_max=-100000;
    for(int i=0; i < 2; ++i){
      if(points[i].x()<x_min)
        x_min = points[i].x();
      if(points[i].x()>x_max)
        x_max = points[i].x();
      if(points[i].y()<y_min)
        y_min = points[i].y();
      if(points[i].y()>y_max)
        y_max = points[i].y();
      if(points[i].z()<z_min)
        z_min = points[i].z();
      if(points[i].z()>z_max)
        z_max = points[i].z();
    }
    _bounding_boxes[i] = std::make_pair(Eigen::Vector3f(x_min,y_min,z_min),Eigen::Vector3f(x_max,y_max,z_max));
    _detections[i]._type = model._type;
  }
}

void ObjectDetector::computeImageBoundingBoxes(){

  for(int r=0; r<_rows; ++r){
    const cv::Vec3f* point_ptr=_points_image.ptr<const cv::Vec3f>(r);
    for(int c=0; c<_cols; ++c, ++point_ptr){
      const cv::Vec3f& p = *point_ptr;

      if(cv::norm(p) < 1e-3)
        continue;

      const Eigen::Vector3f point(p[1],p[0],p[2]);

      for(int j=0; j < _bounding_boxes.size(); ++j){
        int &r_min = _detections[j]._top_left.x();
        int &c_min = _detections[j]._top_left.y();
        int &r_max = _detections[j]._bottom_right.x();
        int &c_max = _detections[j]._bottom_right.y();

        if(inRange(point,_bounding_boxes[j])){
          if(r < r_min)
            r_min = r;
          if(r > r_max)
            r_max = r;

          if(c < c_min)
            c_min = c;
          if(c > c_max)
            c_max = c;

          _detections[j]._pixels.push_back(Eigen::Vector2i(c,r));
          _rgb_image.at<cv::Vec3b>(c,r) = cv::Vec3b(255,0,0);

          break;
        }
      }
    }
  }
}

void ObjectDetector::compute(){
  //Compute world bounding boxes
  double cv_wbb_time = (double)cv::getTickCount();
  computeWorldBoundingBoxes();
  printf("Computing WBB took: %f\n",((double)cv::getTickCount() - cv_wbb_time)/cv::getTickFrequency());

  //Compute image bounding boxes
  double cv_ibb_time = (double)cv::getTickCount();
  computeImageBoundingBoxes();
  printf("Computing IBB took: %f\n",((double)cv::getTickCount() - cv_ibb_time)/cv::getTickFrequency());

  computeLabelImage();
}

constexpr unsigned int str2int(const char* str, int h = 0){
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

float ObjectDetector::type2float(std::string type){
  float step = 1.0f/6.0f;

  switch (str2int(type.c_str())){
    case str2int("table"):
      return 1.0f*step;
    case str2int("chair"):
      return 2.0f*step;
    case str2int("couch"):
      return 3.0f*step;
    case str2int("cabinet"):
      return 4.0f*step;
    case str2int("tomato"):
      return 5.0f*step;
    case str2int("milk"):
      return 6.0f*step;
    default:
      return 0;
  }
}

void ObjectDetector::computeLabelImage(){
  for(int i=0; i < _detections.size(); ++i){
    std::string type = _detections[i].type();
    std::string cropped_type = type.substr(0,type.find_first_of("_"));
    for(int j=0; j < _detections[i]._pixels.size(); ++j){
      int r = _detections[i]._pixels[j].x();
      int c = _detections[i]._pixels[j].y();

      _label_image.at<unsigned char>(r,c) = type2float(cropped_type)*255.0f;
    }
  }
}
