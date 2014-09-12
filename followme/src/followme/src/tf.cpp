#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cstdio>
void get_ground_coeffs() {
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
	static ros::Time last_tf = ros::Time(0);
	static int counter1, counter2;
    try {
        listener.waitForTransform("/odom_combined", "/head_mount_kinect_rgb_optical_frame",
                                ros::Time(0), ros::Duration(1.));
        listener.lookupTransform("/odom_combined", "/head_mount_kinect_rgb_optical_frame",
                                ros::Time(0), transform);
		if (transform.stamp_ == last_tf)
			return;
		last_tf = transform.stamp_;
		double roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw);
		const auto &origin = transform.getOrigin();
		const auto &time = transform.stamp_;
		roll *= 180 / M_PI;
		pitch *= 180 / M_PI;
		yaw *= 180 / M_PI;
		fprintf(stderr, "%u.%09u x=%4.1f y=%4.1f z=%4.1f roll=%5.1f pitch=%5.1f yaw=%5.1f\n",
			time.sec, time.nsec, origin.x(), origin.y(), origin.z(), roll, pitch, yaw); 
    } catch (tf::TransformException ex) {
        if (counter2++ % 100 == 0) ROS_ERROR("%s",ex.what());
        return;
    }
#if 0
    double tanroll = tan(roll + roll_off / 180*M_PI);
    ground_coeffs_mutex.lock();
    ground_coeffs[0] = 0;
    ground_coeffs[1] = -tanroll / sqrt(tanroll*tanroll+1);
    ground_coeffs[2] = -1 / sqrt(tanroll*tanroll+1);
    ground_coeffs[3] = transform.getOrigin().z() + z_off;
    ground_coeffs_mutex.unlock();
    if (ground_coeffs[1] != last_ground_coeffs[1]) {
        std::cerr << "new ground coeffs " << ground_coeffs.transpose() << std::endl;
        last_ground_coeffs = ground_coeffs;
    }
#endif
}


int
main (int argc, char** argv)
{
	ros::init (argc, argv, "tf_test");
    while (ros::ok()) {
        get_ground_coeffs();
    }
	return 0;
}
