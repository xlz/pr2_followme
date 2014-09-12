#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;

void 
cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& input)
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing
//  ROS_INFO("cloud %dx%d", input->width, input->height);
tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Vector3 zaxis(0,0,1);

    try{
	ros::Time start = ros::Time::now();
      listener.waitForTransform("/base_link", "/head_mount_kinect_rgb_optical_frame",
                               ros::Time(0), ros::Duration(1.));
      ROS_INFO("duration: %f", ros::Time::now().toSec() - start.toSec());
      listener.lookupTransform("/base_link", "/head_mount_kinect_rgb_optical_frame",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    double roll, junk;
    transform.getBasis().getRPY(roll, junk, junk);
    double tanroll = tan(roll);
    double b = tanroll / sqrt(tanroll*tanroll+1);
    double c = 1 / sqrt(tanroll*tanroll+1);
    double d = -transform.getOrigin().z();
    ROS_INFO("0*x + %.2f*y + %.2f*z + %.2f = 0", b, c, d);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGBA> >("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
