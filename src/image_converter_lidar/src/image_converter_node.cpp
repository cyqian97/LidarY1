#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "perception_msgs/yolo_box.h"
#include "perception_msgs/yolo_boxes.h"

static const std::string OPENCV_WINDOW = "Image window";
ros::Subscriber pc2_sub;
// typedef const boost::function< void(const sensor_msgs::PointCloud2 &)>  callback;

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_bboxes_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = nullptr;
  boost::shared_ptr<std::vector<perception_msgs::yolo_box>> bboxes = nullptr;

public:


  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera_center_hd/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    sub_bboxes_ = nh_.subscribe("/target_detection", 1, 
    &ImageConverter::updateBBoxes, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void onPC2(const sensor_msgs::PointCloud2Ptr& msg)
  {
    
    ROS_INFO_STREAM("Received point clouds.");
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *_cloud);
    cloud = _cloud;
    
    ROS_INFO_STREAM("Cloud size: " << cloud->size());
  }

  void updateBBoxes(
      const boost::shared_ptr<const perception_msgs::yolo_boxes> bboxes_msg) 
  { 
    std::vector<perception_msgs::yolo_box> _bboxes;

    for(const auto& bbox: bboxes_msg->bounding_boxes) _bboxes.push_back(bbox);

    bboxes = boost::make_shared<std::vector<perception_msgs::yolo_box>>(_bboxes);

    ROS_INFO_STREAM("Get " << bboxes->size() << " bboxes.");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream

    // for(const pcl::PointXYZ& pt: cloud->points)
    if(cloud != nullptr)
      for(const pcl::PointXYZ& pt: cloud->points)
      {
        cv::circle(cv_ptr->image, cv::Point(pt.x,pt.y), 2, CV_RGB(255,0,0));
        // ROS_INFO_STREAM("Point at " << pt.y << ", " << pt.x);
      }

    if(bboxes != nullptr)
      for(const auto& bbox: *bboxes)
        cv::rectangle(cv_ptr->image,cv::Point(bbox.xmin+1000,bbox.ymin+1000),cv::Point(bbox.xmax+1000,bbox.ymax+1000),CV_RGB(255,0,0),3);


    //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window

    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 0.5, 0.5);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh = ros::NodeHandle();
  ImageConverter ic;
  pc2_sub = nh.subscribe("/cepton/distort", 1, &ImageConverter::onPC2, &ic);
  
  ros::spin();
  return 0;
}