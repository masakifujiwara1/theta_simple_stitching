#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
 
class theta_simple_stitching{
public:
  theta_simple_stitching();
  void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat map_x_;
  cv::Mat map_y_;
  int method_;
  double vertex_;
  double src_cx_;
  double src_cy_;
  double src_r_;
  double src_cx2_;
};

theta_simple_stitching::theta_simple_stitching() :
    nh_(),
    it_(nh_),
    pnh_("~"),
    method_(0),
    vertex_(540),
    src_cx_(319),
    src_cy_(319),
    src_r_(283),
    src_cx2_(1280 - src_cx_)
  {
    image_sub_ = it_.subscribe("/image_raw", 1, &theta_simple_stitching::imageCb, this);
    image_pub_ = it_.advertise("/image/mercator", 1);

    map_x_ = cv::Mat::zeros(vertex_, vertex_*2, CV_32FC1);
    map_y_ = cv::Mat::zeros(vertex_, vertex_*2, CV_32FC1);
    for(int y; y < vertex_; y++){
      for(int x; x < vertex_; x++){
        double phi1 = M_PI * x / vertex_;
        double theta1 = M_PI * y / vertex_;
        double X = std::sin(theta1) * std::cos(phi1);
        double Y = std::sin(theta1) * std::sin(phi1);
        double Z = std::cos(theta1);
        double phi2 = std::acos(-X);
        double sign = 0;
        if(Y > 0) sign = 1;
        else if(Y < 0) sign = -1;
        double theta2 = sign * std::acos(-Z/std::sqrt(Y*Y + Z*Z));
        double r_ = 0;
        if(phi2 < M_PI_2){
          switch(method_){
            case 0:
              r_ = phi2 / M_PI * 2;
              break;
            case 1:
              r_ = std::tan(phi2 / 2);
              break;
            case 2:
              r_ = 1 - std::tan((M_PI_2 - phi2) / 2);
              break;
            case 3:
              r_ = std::sin(phi2);
              break;
            case 4:
              r_ = 1 - std::sin(M_PI_2 - phi2);
              break;
          }
          cv::Vec3b *src;
          src = map_x_.ptr<cv::Vec3b>(y);
          src[x] = src_r_ * r_ * std::cos(M_PI - theta2) + src_cx2_;
          src = map_y_.ptr<cv::Vec3b>(y);
          src[x] = src_r_ * r_ * std::sin(M_PI - theta2) + src_cx2_;
        }
        else{
          switch(method_){
            case 0:
              r_ = (M_PI - phi2) / M_PI * 2;
              break;
            case 1:
              r_ = std::tan(M_PI - phi2 / 2);
              break;
            case 2:
              r_ = 1 - std::tan((M_PI_2 + phi2) / 2);
              break;
            case 3:
              r_ = std::sin(M_PI + phi2);
              break;
            case 4:
              r_ = 1 - std::sin(-M_PI_2 + phi2);
              break;
          }
          cv::Vec3b *src;
          src = map_x_.ptr<cv::Vec3b>(y);
          src[x] = src_r_ * r_ * std::cos(M_PI - theta2) + src_cx2_;
          src = map_y_.ptr<cv::Vec3b>(y);
          src[x] = src_r_ * r_ * std::sin(M_PI - theta2) + src_cx2_;
        }
        ROS_INFO("finish init for theta siimple stitching");
      }
    }
  }

  void theta_simple_stitching::imageCb(const sensor_msgs::ImageConstPtr& msg_ptr){
    cv::Mat cv_image;
    try{
      cv_image = cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat resized_image;
    cv::resize(cv_image, resized_image, cv::Size(1280, 720), cv::INTER_LINEAR);

    cv::Mat result_image;
    cv::remap(resized_image, result_image, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();
    image_pub_.publish(pub_msg);
}

 
int main(int argc, char** argv){
  ros::init(argc, argv, "theta_simple_stitching");
  theta_simple_stitching ss;
  ros::spin();
  return 0;
}
