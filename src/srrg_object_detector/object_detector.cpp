#include "object_detector.h"
#include <iomanip>


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
                     0.02f,
                     8.0f);

  srrg_core::Float3Image cross_normals_image;
  computeSimpleNormals(cross_normals_image,
                       _points_image,
                       3,
                       3,
                       8);


  for(int r=0; r<_rows; ++r){
    const cv::Vec3f* point_ptr = _points_image.ptr<const cv::Vec3f>(r);
    const cv::Vec3f* normal_ptr = cross_normals_image.ptr<const cv::Vec3f>(r);
    for(int c=0; c<_cols; ++c, ++point_ptr, ++normal_ptr){
      const cv::Vec3f& p = *point_ptr;
      const cv::Vec3f& n = *normal_ptr;

      if(cv::norm(p) < 1e-3 || cv::norm(n) < 1e-3)
        continue;

      const Eigen::Vector3f point(p[0],p[1],p[2]);
      const Eigen::Vector3f normal(n[0],n[1],n[2]);

      _depth_cloud.push_back(RichPoint3D(point,normal,1.0f,Eigen::Vector3f(0.5,0.5,1.0)));
    }

//    std::ofstream outfile;
//    outfile.open("depth.cloud");
//    _depth_cloud.write(outfile);
//    outfile.close();

  }

  _label_image.create(_rows,_cols);
  _label_image=cv::Vec3b(0,0,0);
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
  _bounding_boxes.resize(num_models);
  _detections.resize(num_models);
  for(int i=0; i<num_models; ++i){
    _detections[i] = DetectionPtr (new Detection);
  }

  std::cerr << "Detected " << num_models << " models" << std::endl << std::endl;
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

  for(int i=0; i<_models.size(); ++i){
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
    _detections[i]->type() = model._type;
  }
}

void ObjectDetector::computeImageBoundingBoxes(){

  for(int r=0; r<_rows; ++r){
    const cv::Vec3f* point_ptr=_points_image.ptr<const cv::Vec3f>(r);
    for(int c=0; c<_cols; ++c, ++point_ptr){
      const cv::Vec3f& p = *point_ptr;

      if(cv::norm(p) < 1e-3)
        continue;

      const Eigen::Vector3f point(p[0],p[1],p[2]);

      for(int j=0; j < _bounding_boxes.size(); ++j){

        if(inRange(point,_bounding_boxes[j])){
          if(r < _detections[j]->topLeft().x())
            _detections[j]->topLeft().x() = r;
          if(r > _detections[j]->bottomRight().x())
            _detections[j]->bottomRight().x() = r;

          if(c < _detections[j]->topLeft().y())
            _detections[j]->topLeft().y() = c;
          if(c > _detections[j]->bottomRight().y())
            _detections[j]->bottomRight().y() = c;

          _detections[j]->pixels()[_detections[j]->size()] = Eigen::Vector2i(c,r);
          ++(_detections[j]->size());
          break;
        }
      }
    }
  }

  for(size_t i=0; i < _detections.size(); ++i){
    _detections[i]->pixels().resize(_detections[i]->size());
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


cv::Vec3b ObjectDetector::type2color(std::string type){
  int c;

  if(type == "table")
    c = 1;
  if(type == "tomato")
    c = 2;
  if(type == "salt")
    c = 3;
  if(type == "milk")
    c = 4;

  std::stringstream stream;
  stream << std::setw(6) << std::setfill('0') << std::hex << ((float)c/4.0f)*16777215;
  std::string result(stream.str());

  unsigned long r_value = std::strtoul(result.substr(0,2).c_str(), 0, 16);
  unsigned long g_value = std::strtoul(result.substr(2,2).c_str(), 0, 16);
  unsigned long b_value = std::strtoul(result.substr(4,2).c_str(), 0, 16);

  cv::Vec3b color(r_value,g_value,b_value);
  return color;

}

void ObjectDetector::computeLabelImage(){
  for(int i=0; i < _detections.size(); ++i){
    std::string type = _detections[i]->type();
    std::string cropped_type = type.substr(0,type.find_first_of("_"));
    cv::Vec3b color = type2color(cropped_type);
    for(int j=0; j < _detections[i]->pixels().size(); ++j){
      int r = _detections[i]->pixels()[j].x();
      int c = _detections[i]->pixels()[j].y();

      _label_image.at<cv::Vec3b>(c,r) = color;
    }
  }
}
