#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "geometry_msgs/Point.h"
#include "ground_projection/GetGroundCoord.h"
#include "ground_projection/GetImageCoord.h"
#include "ground_projection/EstimateHomography.h"
#include "duckietown_msgs/Pixel.h"
#include "duckietown_msgs/Vector2D.h"
#include "duckietown_msgs/Segment.h"
#include "duckietown_msgs/SegmentList.h"
#include <std_srvs/Empty.h>

namespace enc = sensor_msgs::image_encodings;

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class GroundProjection
{
  ros::NodeHandle nh_;
  ros::ServiceServer service_homog_;
  ros::ServiceServer service_gnd_coord_;
  ros::ServiceServer service_img_coord_;
  ros::Subscriber sub_lineseglist_;
  ros::Publisher pub_lineseglist_;
  sensor_msgs::CameraInfo::ConstPtr camera_info_;
  image_geometry::PinholeCameraModel pcm_;
  cv::Mat H_, Hinv_;
  bool rectified_input_;

  std::string node_name_;

  std::string config_name_;
  std::string config_file_name_;
  std::string h_file_;

public:
  GroundProjection()
  : nh_("~")
  {

    node_name_ = ros::this_node::getName();

    std::string DT = ros::package::getPath("duckietown");
    // load from homography yaml file
    nh_.param<std::string>("config", config_name_, "baseline");
    nh_.param<std::string>("config_file_name", config_file_name_, "default");
    h_file_ = DT
             + "/config/" + config_name_ +
             "/calibration/camera_extrinsic/"
             + config_file_name_ + ".yaml";
    std::ifstream fin(h_file_.c_str());
    if (!fin.good())
    {
      ROS_WARN_STREAM("Can't find homography file: " << h_file_ <<
                      " Using default calibration instead.");
      h_file_ = DT + "/config/" + config_name_ + "/calibration/camera_extrinsic/default.yaml";
      // h_file_ = get_package_filename("package://ground_projection/homography/default.yaml");
    }
    ROS_INFO("load from homography yaml file [%s].", h_file_.c_str());

    std::vector<float> h;
    h.resize(9);
    read_homography_yaml(h_file_, h);

    // update homography matrix (H_)
    H_.create(3, 3, CV_32F);
    for(int r=0; r<3; r++)
    {
      for(int c=0; c<3; c++)
      {
        H_.at<float>(r, c) = h[r*3+c];
      }
    }
    // estimate inv(H_)
    Hinv_ = H_.inv();

    nh_.param<bool>("rectified_input", rectified_input_, false);

    std::string camera_info_topic = nh_.resolveName("camera_info");
    ROS_INFO("[%s] Waiting for message on topic: %s", node_name_.c_str(),camera_info_topic.c_str());
    camera_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, nh_);
    ROS_INFO("[%s] Got camera_info", node_name_.c_str());

    pcm_.fromCameraInfo(camera_info_);

    // Publisher
    pub_lineseglist_ = nh_.advertise<duckietown_msgs::SegmentList>("lineseglist_out", 1);

    // Ready the services
    service_homog_ = nh_.advertiseService("estimate_homography", &GroundProjection::estimate_homography_cb, this);
    ROS_INFO("estimate_homography is ready.");
    service_gnd_coord_ = nh_.advertiseService("get_ground_coordinate", &GroundProjection::get_ground_coordinate_cb, this);
    ROS_INFO("get_ground_coordinate is ready.");
    service_img_coord_  = nh_.advertiseService("get_image_coordinate", &GroundProjection::get_image_coordinate_cb, this);
    ROS_INFO("get_image_coordinate is ready.");

    // Subscriber
    sub_lineseglist_ = nh_.subscribe("lineseglist_in", 1, &GroundProjection::lineseglist_cb, this);
  }

  ~GroundProjection()
  {
  }

private:
  duckietown_msgs::Pixel vector2pixel(const duckietown_msgs::Vector2D& vec)
  {
    float w = static_cast<float>(camera_info_->width);
    float h = static_cast<float>(camera_info_->height);

    duckietown_msgs::Pixel pixel;

    pixel.u = static_cast<int>(w * vec.x);
    pixel.v = static_cast<int>(h * vec.y);

    // boundary check (might be redundant, but just in case)
    if(pixel.u < 0)     pixel.u = 0;
    if(pixel.u > w-1)   pixel.u = w-1;
    if(pixel.v < 0)     pixel.v = 0;
    if(pixel.v > h-1)   pixel.v = h-1;

    return pixel;
  }

  duckietown_msgs::Vector2D pixel2vector(const duckietown_msgs::Pixel pixel)
  {
    float w = static_cast<float>(camera_info_->width);
    float h = static_cast<float>(camera_info_->height);

    duckietown_msgs::Vector2D vec;

    vec.x = static_cast<float>(pixel.u)/w;
    vec.y = static_cast<float>(pixel.v)/h;

    return vec;
  }

  void getRectifiedImage(const sensor_msgs::Image::ConstPtr img_msg, cv::Mat &mat_rect)
  {
    // Convert to cv::Mat
    // cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg);
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg,enc::MONO8);
    pcm_.rectifyImage(cv_img->image,mat_rect);
  }

  void image2ground(const duckietown_msgs::Vector2D& vec, geometry_msgs::Point& point)
  {
    duckietown_msgs::Pixel pixel = vector2pixel(vec);
    image2ground(pixel, point);
  }

  void ground2image(const geometry_msgs::Point& point, duckietown_msgs::Vector2D& vec)
  {
    duckietown_msgs::Pixel pixel;
    ground2image(point, pixel);
    vec = pixel2vector(pixel);
  }

  void image2ground(const duckietown_msgs::Pixel& pixel, geometry_msgs::Point& point)
  {
    cv::Point3f pt_img(static_cast<float>(pixel.u), static_cast<float>(pixel.v), 1.f);

    if(!rectified_input_)
    {
      // point-wise undistortion
      cv::Point2d pt_undistorted = pcm_.rectifyPoint(
        cv::Point2d(static_cast<double>(pt_img.x), static_cast<double>(pt_img.y))
      );

      pt_img.x = static_cast<float>(pt_undistorted.x);
      pt_img.y = static_cast<float>(pt_undistorted.y);
    }

    cv::Mat pt_gnd_ = H_ * cv::Mat(pt_img);
    cv::Point3f pt_gnd(pt_gnd_);

    point.x = pt_gnd.x/pt_gnd.z;
    point.y = pt_gnd.y/pt_gnd.z;
    point.z = 0.f;
  }

  void ground2image(const geometry_msgs::Point& point, duckietown_msgs::Pixel& pixel)
  {
    cv::Point3f pt_gnd(static_cast<float>(point.x), static_cast<float>(point.y), 1.f);

    cv::Mat pt_img_ = Hinv_ * cv::Mat(pt_gnd);
    cv::Point3f pt_img(pt_img_);

    pt_img.x /= pt_img.z;
    pt_img.y /= pt_img.z;

    if(!rectified_input_)
    {
      // get distorted image coordinate (pt_distorted) from undistorted one (pt_img)
      cv::Point2d pt_distorted = pcm_.unrectifyPoint(
        cv::Point2d(static_cast<double>(pt_img.x), static_cast<double>(pt_img.y))
      );

      pixel.u = static_cast<int>(pt_distorted.x);
      pixel.v = static_cast<int>(pt_distorted.y);
    }
    else
    {
      pixel.u = static_cast<int>(pt_img.x);
      pixel.v = static_cast<int>(pt_img.y);
    }
  }



  void lineseglist_cb(const duckietown_msgs::SegmentList::ConstPtr& msg)
  {
    duckietown_msgs::SegmentList msg_new = *msg;
    for(int i=0; i<msg_new.segments.size(); i++)
    {
      // each line segment is composed of two end points [0:1]
      image2ground(msg_new.segments[i].pixels_normalized[0], msg_new.segments[i].points[0]);
      image2ground(msg_new.segments[i].pixels_normalized[1], msg_new.segments[i].points[1]);
    }
    pub_lineseglist_.publish(msg_new);
  }

  bool get_ground_coordinate_cb(ground_projection::GetGroundCoord::Request &req,
                                ground_projection::GetGroundCoord::Response &res)
  {
    image2ground(req.normalized_uv, res.gp);

    ROS_INFO("normalized image coord (norm u=%f, norm v=%f), ground coord (x=%f, y=%f, z=%f)", req.normalized_uv.x, req.normalized_uv.y, res.gp.x, res.gp.y, res.gp.z);
    return true;
  }

  bool get_image_coordinate_cb(ground_projection::GetImageCoord::Request &req,
                                ground_projection::GetImageCoord::Response &res)
  {
    ground2image(req.gp, res.normalized_uv);

    ROS_INFO("ground coord (x=%f, y=%f, z=%f), normalized image coord (norm u=%f, norm v=%f)", req.gp.x, req.gp.y, req.gp.z, res.normalized_uv.x, res.normalized_uv.y);
    return true;
  }


  bool estimate_homography_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    ROS_INFO("Estimating homography...");
    ROS_INFO_STREAM("Waiting from raw image on topic: " << nh_.resolveName("cali_image"));
    sensor_msgs::Image::ConstPtr img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("cali_image", nh_);
    ROS_INFO_STREAM("Received raw image on topic: " << nh_.resolveName("cali_image"));

    cv::Mat mat_rect;
    cv::Mat mat_homography;
    getRectifiedImage(img_msg,mat_rect);
    ROS_INFO("Raw image rectifed.");

    if(estimate_homography(mat_rect,mat_homography)){
      ROS_INFO("Homography estimated.");
    }
    else{
      ROS_WARN("Homography estimation failed.");
      return false;
    }

    mat_homography.copyTo(H_);
    std::vector<float> vec_homography(9,0);
    // Fill homography vector
    for(int r=0; r<3; r++)
    {
      for(int c=0; c<3; c++)
      {
        vec_homography[r*3+c] = mat_homography.at<float>(r, c);
      }
    }
    // Write to yaml file
    std::string wrtie_file_path = ros::package::getPath("duckietown")
            + "/config/" +
            config_name_ +
             "/calibration/camera_extrinsic" + ros::this_node::getNamespace() + ".yaml";
    ROS_INFO("Homography yaml file [%s].", wrtie_file_path.c_str());
    write_homography_yaml(wrtie_file_path, vec_homography);
    return true;
  }

  bool estimate_homography(cv::Mat& Ir, cv::Mat& H)
  {
    // estimate homography
    int board_w, board_h;
    float square_size;
    nh_.param<int>("board_w", board_w, 7);
    nh_.param<int>("board_h", board_h, 5);
    nh_.param<float>("square_size", square_size, 0.031f);

    int board_n = board_w * board_h;

    cv::Size board_size(board_w, board_h);
    std::vector<cv::Point2f> corners_;
    bool found = findChessboardCorners(Ir, board_size, corners_);
    if(found)
    {
      cornerSubPix(Ir, corners_, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    else
    {
      ROS_ERROR("Couldn't find a checkerboard");
      return false;
    }

    std::vector<cv::Point2f> pts_gnd_, pts_img_;
    pts_gnd_.resize(board_w*board_h);
    pts_img_.resize(board_w*board_h);

    float x_offset, y_offset;
    nh_.param<float>("x_offset", x_offset, 0.191f);
    nh_.param<float>("y_offset", y_offset, -0.093f);
    cv::Point2f offset = cv::Point2f(x_offset, y_offset);

    cv::Point2f corner_lr = corners_[0]; // lower-right point
    cv::Point2f corner_ur  = corners_[(board_h-1)*(board_w)]; // upper-right point
    cv::Point2f corner_ul  = corners_[(board_h)*(board_w)-1]; // upper-left point

    // std::cout << "corner_lr: " << corner_lr.x << ", " << corner_lr.y << std::endl;
    // std::cout << "corner_ur: " << corner_ur.x << ", " << corner_ur.y << std::endl;
    // std::cout << "corner_ul: " << corner_ul.x << ", " << corner_ul.y << std::endl;

    bool h_flipped = false, v_flipped = false;
    if(corner_ul.x > corner_ur.x)
    {
      // rows are horizontally flipped
      h_flipped = true;
    }

    if(corner_lr.y < corner_ur.y)
    {
      // cols are vertically flipped
      v_flipped = true;
    }

    for(int r=0; r<board_h; r++)
    {
      for(int c=0; c<board_w; c++)
      {
        pts_gnd_[r*(board_w)+c] = cv::Point2f(float(r)*square_size, float(c)*square_size) + offset;
        pts_img_[r*(board_w)+c] = corners_[(v_flipped ? board_h - 1 - r : r)*(board_w)+(h_flipped ? board_w - 1 - c : c)];
      }
    }

    H = cv::findHomography(pts_img_, pts_gnd_, CV_RANSAC);
    H.convertTo(H, CV_32F);

    return true;
  }


  bool read_homography_yaml(const std::string& h_fname,
                            std::vector<float>& h)
  {
    std::ifstream fin(h_fname.c_str());
    if (!fin.good())
    {
      ROS_INFO("Unable to open homography file [%s]", h_fname.c_str());
      return false;
    }
    // bool success = readCalibrationYml(fin, camera_name, cam_info);
    bool success = read_homography_yaml_(fin, h);

    if (!success)
      ROS_ERROR("Failed to parse homography from file [%s]", h_fname.c_str());
    return success;
  }

  bool read_homography_yaml_(std::istream& fin, std::vector<float>& h)
  {
    try
    {
#ifdef HAVE_NEW_YAMLCPP
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      if (!parser)
      {
        ROS_ERROR("Unable to create YAML parser for homography");
        return false;
      }
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif
      assert(h.size() == 9);

      const YAML::Node& h_data = doc["homography"];
      for(int i = 0; i < 9; ++i)
        h_data[i] >> h[i];

      return true;
    }
    catch (YAML::Exception& e) {
      ROS_ERROR("Exception parsing YAML homography:\n%s", e.what());
      return false;
    }
  }

  bool write_homography_yaml(const std::string& h_fname,
                             const std::vector<float>& h)
  {
    boost::filesystem::path dir(boost::filesystem::path(h_fname).parent_path());
    if(!dir.empty() && !boost::filesystem::exists(dir) &&
      !boost::filesystem::create_directories(dir))
    {
      ROS_ERROR("Unable to create directory for homography file [%s]", dir.c_str());
    }

    std::ofstream out(h_fname.c_str());
    if(!out.is_open())
    {
      ROS_ERROR("Unable to open homography file [%s] for writing", h_fname.c_str());
      return false;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "homography" << YAML::Value << YAML::Flow;

    emitter << YAML::BeginSeq;
    for (int i = 0; i < h.size(); i++)
      emitter << h[i];
    emitter << YAML::EndSeq;

    emitter << YAML::EndMap;

    out << emitter.c_str();
    return true;
  }


  std::string get_package_filename(const std::string &url)
  {
    ROS_DEBUG_STREAM("homography URL: " << url);

    // Scan URL from after "package://" until next '/' and extract
    // package name.
    size_t prefix_len = std::string("package://").length();
    size_t rest = url.find('/', prefix_len);
    std::string package(url.substr(prefix_len, rest - prefix_len));

    // Look up the ROS package path name.
    std::string pkgPath(ros::package::getPath(package));
    if(pkgPath.empty())
    {
      ROS_WARN_STREAM("unknown package: " << package << " (ignored)");
      return pkgPath;
    }
    else
    {
      // Construct file name from package location and remainder of URL.
      return pkgPath + url.substr(rest);
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_projection");

  GroundProjection gp;

  ros::spin();

  return 0;
}
