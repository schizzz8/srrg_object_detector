#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

using namespace srrg_core;

class Model{
public:
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

void readData(char* filename,
              Eigen::Isometry3f& rgbd_camera_transform,
              Eigen::Isometry3f& logical_camera_transform,
              std::vector<Model>& models);

int main(int argc, char** argv){

  //read images
  cv::Mat rgb_image = cv::imread(argv[1],cv::IMREAD_UNCHANGED);
  cv::Mat raw_depth_image = cv::imread(argv[2],cv::IMREAD_UNCHANGED);

  int rows = rgb_image.rows;
  int cols = rgb_image.cols;

  //compute depth point cloud
  Eigen::Matrix3f K;
  K << 277.127, 0.0, 160.5, 0.0, 277.127, 120.5, 0.0, 0.0, 1.0;

  srrg_core::FloatImage depth_image;
  srrg_core::Float3Image directions_image;
  srrg_core::Float3Image points_image;

  directions_image.create(rows,cols);
  initializePinholeDirections(directions_image,K);
  points_image.create(rows,cols);
  convert_16UC1_to_32FC1(depth_image, raw_depth_image);
  computePointsImage(points_image,
                     directions_image,
                     depth_image,
                     0.04f,
                     8.0f);

  //read robot pose and logical image
  Eigen::Isometry3f rgbd_camera_transform=Eigen::Isometry3f::Identity();
  Eigen::Isometry3f logical_camera_transform=Eigen::Isometry3f::Identity();
  std::vector<Model> models;
  readData(argv[3],rgbd_camera_transform,logical_camera_transform,models);

  cv::imshow("rgb",rgb_image);
  cv::imshow("depth",depth_image/5.0f);
  cv::waitKey();

  return 0;
}


void readData(char* filename,
              Eigen::Isometry3f& rgbd_camera_transform,
              Eigen::Isometry3f& logical_camera_transform,
              std::vector<Model>& models){

  std::string line;
  std::ifstream data(filename);

  if(data.is_open()) {
    if(std::getline(data,line)) {
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      rgbd_camera_transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      rgbd_camera_transform.linear().matrix() = R;
    }
    if(std::getline(data,line)) {
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      logical_camera_transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      logical_camera_transform.linear().matrix() = R;
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
      models.push_back(model);
    }
    data.close();
  }

  std::cerr << "RGBD camera pose:" << std::endl;
  std::cerr << rgbd_camera_transform.translation().x() << " "
            << rgbd_camera_transform.translation().y() << " "
            << rgbd_camera_transform.translation().z() << " ";

  const Eigen::Matrix3f rgbd_camera_rotation = rgbd_camera_transform.linear().matrix();
  std::cerr << rgbd_camera_rotation(0,0) << " "
            << rgbd_camera_rotation(0,1) << " "
            << rgbd_camera_rotation(0,2) << " "
            << rgbd_camera_rotation(1,0) << " "
            << rgbd_camera_rotation(1,1) << " "
            << rgbd_camera_rotation(1,2) << " "
            << rgbd_camera_rotation(2,0) << " "
            << rgbd_camera_rotation(2,1) << " "
            << rgbd_camera_rotation(2,2) << std::endl;

  std::cerr << "Logical camera pose:" << std::endl;
  std::cerr << logical_camera_transform.translation().x() << " "
            << logical_camera_transform.translation().y() << " "
            << logical_camera_transform.translation().z() << " ";

  const Eigen::Matrix3f logical_camera_rotation = logical_camera_transform.linear().matrix();
  std::cerr << logical_camera_rotation(0,0) << " "
            << logical_camera_rotation(0,1) << " "
            << logical_camera_rotation(0,2) << " "
            << logical_camera_rotation(1,0) << " "
            << logical_camera_rotation(1,1) << " "
            << logical_camera_rotation(1,2) << " "
            << logical_camera_rotation(2,0) << " "
            << logical_camera_rotation(2,1) << " "
            << logical_camera_rotation(2,2) << std::endl;

  int num_models=models.size();
  std::cerr << num_models << std::endl;

  std::cerr << "Detected " << num_models << " models" << std::endl;
  for(int i=0; i<num_models; ++i){
    const Model &model = models[i];
    std::cerr << model._type << " ";
    const Eigen::Isometry3f model_transform=model._pose;
    std::cerr << model_transform.translation().x() << " "
              << model_transform.translation().y() << " "
              << model_transform.translation().z() << " ";

    const Eigen::Matrix3f model_rotation = model_transform.linear().matrix();
    std::cerr << model_rotation(0,0) << " "
              << model_rotation(0,1) << " "
              << model_rotation(0,2) << " "
              << model_rotation(1,0) << " "
              << model_rotation(1,1) << " "
              << model_rotation(1,2) << " "
              << model_rotation(2,0) << " "
              << model_rotation(2,1) << " "
              << model_rotation(2,2) << " ";

    std::cerr << model._min.x() << " "
              << model._min.y() << " "
              << model._min.z() << " "
              << model._max.x() << " "
              << model._max.y() << " "
              << model._max.z() << std::endl;

  }

}
