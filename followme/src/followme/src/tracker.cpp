#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include "roscpp/Empty.h"

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/visualization/vtk.h>
void getTFGroundCoeffs(Eigen::VectorXf &v);
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>
#include <boost/thread.hpp>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::string target_frame;
boost::mutex tf_ground_coeffs_mutex;
Eigen::VectorXf tf_ground_coeffs, last_tf_ground_coeffs;
bool tf_ground_coeffs_changed = false;
double roll_off = 1.8, z_off = 0.03;//tf not calibrated?
void calculate_tf_ground_coeffs(const tf::StampedTransform &transform) {
	double roll, junk;
	transform.getBasis().getRPY(roll, junk, junk);
	double tanroll = tan(roll + roll_off / 180*M_PI);
	tf_ground_coeffs_mutex.lock();
	tf_ground_coeffs[0] = 0;
	tf_ground_coeffs[1] = -tanroll / sqrt(tanroll*tanroll+1);
	tf_ground_coeffs[2] = -1 / sqrt(tanroll*tanroll+1);
	tf_ground_coeffs[3] = transform.getOrigin().z() + z_off;
	tf_ground_coeffs_mutex.unlock();
	if ((tf_ground_coeffs - last_tf_ground_coeffs).squaredNorm() > 1e-4) {
		std::cerr << "new ground coeffs " << tf_ground_coeffs.transpose() << std::endl;
		last_tf_ground_coeffs = tf_ground_coeffs;
		tf_ground_coeffs_changed = true;
	}
}
void get_tf_ground_coeffs_once() {
	static tf::TransformListener listener;
	static tf::StampedTransform transform;
	static ros::Time last_tf = ros::Time(0);
	try {
		listener.waitForTransform(target_frame, "/head_mount_kinect_rgb_optical_frame",
								ros::Time(0), ros::Duration(1.));
		listener.lookupTransform(target_frame, "/head_mount_kinect_rgb_optical_frame",
								ros::Time(0), transform);
		if (transform.stamp_ == last_tf)
			return;
		last_tf = transform.stamp_;
		calculate_tf_ground_coeffs(transform);
	} catch (tf::TransformException ex) {
		//ROS_ERROR("%s",ex.what());
		return;
	}
}
void tf_ground_coeffs_callback() {
	while (ros::ok()) {
		get_tf_ground_coeffs_once();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
}

void getTFGroundCoeffs(Eigen::VectorXf &v) {
	tf_ground_coeffs_mutex.lock();
	v = tf_ground_coeffs;
	tf_ground_coeffs_changed = false;
	tf_ground_coeffs_mutex.unlock();
}
#if 0
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* junk) {
	if (!event.keyDown())
		return;
	const auto &key = event.getKeySym();
	if (key == "y") {
		roll_off += 0.05;
	} else if (key == "t") {
		roll_off -= 0.05;
	} else if (key == "b") {
		z_off += 0.01;
	} else if (key == "v") {
		z_off -= 0.01;
	} else {
		return;
	}
    std::cerr << "roll_off=" << roll_off << " z_off=" << z_off << std::endl;
}
#endif
boost::mutex camera_transform_mutex;
tf::StampedTransform cameraTransform;
void get_camera_transform() {
	static ros::Time last_tf = ros::Time(0);
	static tf::TransformListener listener;
	try {
		listener.waitForTransform(target_frame, "/head_mount_kinect_rgb_optical_frame",
								ros::Time(0), ros::Duration(1.));
		camera_transform_mutex.lock();
		listener.lookupTransform(target_frame, "/head_mount_kinect_rgb_optical_frame",
								ros::Time(0), cameraTransform);
		camera_transform_mutex.unlock();
#if 0
		if (cameraTransform.stamp_ == last_tf)
			return;
		last_tf = cameraTransform.stamp_;
		double roll, pitch, yaw;
		cameraTransform.getBasis().getRPY(roll, pitch, yaw);
		const auto &origin = cameraTransform.getOrigin();
		const auto &time = cameraTransform.stamp_;
		roll *= 180 / M_PI;
		pitch *= 180 / M_PI;
		yaw *= 180 / M_PI;
		tf::Vector3 v(0, 0, 1);
		v = cameraTransform * v;
		fprintf(stderr, "x=%.2f y=%.2f z=%.2f\n", v.x(), v.y(), v.z());
		fprintf(stderr, "%u.%09u x=%5.2f y=%5.2f z=%5.2f roll=%5.1f pitch=%5.1f yaw=%5.1f\n",
			time.sec, time.nsec, origin.x(), origin.y(), origin.z(), roll, pitch, yaw);
#endif
	} catch (tf::TransformException ex) {
		//ROS_ERROR("%s",ex.what());
		return;
	}
}
void camera_transform_callback() {
	while (ros::ok()) {
		get_camera_transform();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
}

boost::mutex cloud_mutex;
void cloud_callback(const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud, bool* new_cloud_available_flag, double *new_cloud_timestamp) {
	cloud_mutex.lock();
	*cloud = *callback_cloud;
	*new_cloud_available_flag = true;
	*new_cloud_timestamp = pcl::getTime();
	cloud_mutex.unlock();
}

#include <unordered_map>
struct Tracker {
	typedef struct { double time; double confidence; Eigen::Vector2f pos; } Sample;
	typedef std::deque<Sample> Trajectory;
	std::unordered_map<int, Trajectory> targets;
	int next_target_id;
	int target_focus;
	static constexpr double MAX_DISTANCE = 1.;
	static constexpr double MIN_DISTANCE = 0.04;
	static constexpr double TIMEOUT = 3.;
	static constexpr double FOCUS_TIMEOUT = 10.;
	static constexpr double MIN_CONFIDENCE = -1.;
	static const int TRAJECTORY_SIZE = 100;

	ros::ServiceServer srvFocus, srvDefocus;
	ros::Publisher pubNearest;

	Tracker(ros::NodeHandle &nh) {
		next_target_id = 0;
		target_focus = -1;
		srvFocus = nh.advertiseService("/tracker/focus", &Tracker::srv_focus, this);
		srvDefocus = nh.advertiseService("/tracker/defocus", &Tracker::srv_defocus, this);
		pubNearest = nh.advertise<geometry_msgs::Point>("/tracker/nearest", 10);
	}
	bool srv_focus(roscpp::Empty::Request &req, roscpp::Empty::Response &res) {
		target_focus = get_nearest();
		return true;
	}
	bool srv_defocus(roscpp::Empty::Request &req, roscpp::Empty::Response &res) {
		target_focus = -1;
		return true;
	}
	void getRGBFromHue(int hue, double &r, double &g, double &b) {
		double h = (hue % 360) / 60.;
		int i = h;
		double f = h - i;
		double p = 0;
		double q = 1 - f;
		double t = f;
		switch(i) {
			case 0: r = 1; g = t; b = p; break;
			case 1: r = q; g = 1; b = p; break;
			case 2: r = p; g = 1; b = t; break;
			case 3: r = p; g = q; b = 1; break;
			case 4: r = t; g = p; b = 1; break;
			case 5: r = 1; g = p; b = q; break;
		}
	}
	void drawPosition(pcl::visualization::PCLVisualizer &viewer, const Eigen::Vector2f &pos, int id) {
		pcl::ModelCoefficients coeffs;
		coeffs.values.push_back (pos[0]);
		coeffs.values.push_back (pos[1]);
		coeffs.values.push_back (1.75/2);
		// rotation
		coeffs.values.push_back (0.0);
		coeffs.values.push_back (0.0);
		coeffs.values.push_back (0.0);
		coeffs.values.push_back (1.0);
		// size
		coeffs.values.push_back (0.5);
		coeffs.values.push_back (0.5);
		coeffs.values.push_back (1.75);

		std::stringstream name;
		name << "target_" << id;
		viewer.removeShape(name.str());
		viewer.addCube(coeffs, name.str());
		double r = 1, g = 1, b = 1;
		if (id >= 0)
			getRGBFromHue(id*67, r, g, b);
			
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name.str());
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, name.str());
	}
	void drawTrajectory(pcl::visualization::PCLVisualizer &viewer, int id) {
		const auto &traj = targets[id];
		if (traj.size() == 0)
			return;
		pcl::PointCloud<pcl::PointXYZ>::Ptr polygon(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto &sample: traj) {
			polygon->push_back(pcl::PointXYZ(sample.pos[0], sample.pos[1], 0));
		}
		polygon->push_back(pcl::PointXYZ(NAN, NAN, NAN));
		std::stringstream name;
		name << "trajectory_" << id;
		viewer.removeShape(name.str());
		viewer.addPolygon<pcl::PointXYZ>(polygon, name.str());
		double r, g, b;
		getRGBFromHue(id * 67, r, g, b);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
			r, g, b, name.str());
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
			1, name.str());
#if 0
		name << "_start";
		viewer.removeShape(name.str());
		pcl::ModelCoefficients sphere_coeff;
		sphere_coeff.values.resize(4);
		sphere_coeff.values[0] = traj[0].pos[0];
		sphere_coeff.values[1] = traj[0].pos[1];
		sphere_coeff.values[2] = 0
		sphere_coeff.values[3] = 0.05;
		viewer.addSphere(sphere_coeff, name.str());
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
			r, g, b, name.str());
#endif
	}
	void draw(pcl::visualization::PCLVisualizer &viewer, double time) {
		for (const auto &id_traj: targets) {
			const auto &id = id_traj.first;
			const auto &traj = id_traj.second;
			if (traj.size() == 0)
				continue;
			drawTrajectory(viewer, id);
			const auto &sample = traj.back();
			if (sample.time == time && sample.confidence > MIN_CONFIDENCE)
				drawPosition(viewer, traj.back().pos, id);
		}
	}
	int get_nearest() {
		int minId = -1;
		double minDistSq = INFINITY;
		for (auto &id_traj: targets) {
			const auto &id = id_traj.first;
			const auto &traj = id_traj.second;
			if (traj.size() == 0)
				continue;
			double distsq = traj.back().pos.squaredNorm();
			if (distsq < minDistSq) {
				minDistSq = distsq;
				minId = id;
			}
		}
		return minId;
	}
	void publish_nearest() {
		int id;
		if (target_focus != -1)
			id = target_focus;
		else
			id = get_nearest();
		if (id == -1)
			return;
		const auto &pos = targets[id].back().pos;
		geometry_msgs::Point msg;
		msg.x = pos[0];
		msg.y = pos[1];
		msg.z = 0;
		pubNearest.publish(msg);
	}
	void update(double time, const std::vector<Eigen::Vector2f> &measurements, const std::vector<float> &confidences) {
		//each measurement can be: of an existing target, a duplicate, a new target
		std::unordered_map<int, int> nearestTargetOfPos;
		for (int i = 0; i < measurements.size(); i++) {
			int nearestTarget = -1;
			double minDistSq = INFINITY;
			for (const auto &id_traj: targets) {
				const auto &id = id_traj.first;
				const auto &traj = id_traj.second;
				if (traj.size() == 0)
					continue;
				double distsq = (traj.back().pos - measurements[i]).squaredNorm();
				if (distsq < minDistSq) {
					minDistSq = distsq;
					nearestTarget = id;
				}
			}
			nearestTargetOfPos[i] = nearestTarget;
		}
		std::unordered_map<int, int> nearestPosOfTarget;
		for (const auto &id_traj: targets) {
			const auto &id = id_traj.first;
			const auto &traj = id_traj.second;
			if (traj.size() == 0)
				continue;
			int nearestMeasurement = -1;
			double minDistSq = INFINITY;
			for (int i = 0; i < measurements.size(); i++) {
				double distsq = (traj.back().pos - measurements[i]).squaredNorm();
				if (distsq < minDistSq) {
					minDistSq = distsq;
					nearestMeasurement = i;
				}
			}
			nearestPosOfTarget[id] = nearestMeasurement;
		}
		//extend existing trajectories
		for (auto &id_traj: targets) {
			auto &id = id_traj.first;
			auto &traj = id_traj.second;
			int posid = nearestPosOfTarget[id];
			if (posid == -1)
				continue;
			double distsq = (traj.back().pos - measurements[posid]).squaredNorm();
			//all samples are too far away
			if (distsq > MAX_DISTANCE)
				continue;
			//the nearest sample belongs (is nearer) to another target
			if (nearestTargetOfPos[posid] != id)
				continue;
			//extend
			Sample s;
			s.time = time;
			s.pos = measurements[posid];
			traj.push_back(s);
		}
		//create new targets
		for (int i = 0; i < measurements.size(); i++) {
			const auto &pos = measurements[i];
			double minDistSq = INFINITY;
			for (const auto &id_traj: targets) {
				const auto &id = id_traj.first;
				const auto &traj = id_traj.second;
				if (traj.size() == 0)
					continue;
				double distsq = (traj.back().pos - pos).squaredNorm();
				if (distsq < minDistSq) {
					minDistSq = distsq;
				}
			}
			if (minDistSq > MAX_DISTANCE && confidences[i] > MIN_CONFIDENCE) {
				//create
				targets[next_target_id] = std::deque<Sample>(1);
				targets[next_target_id].front().time = time;
				targets[next_target_id].front().pos = pos;
				next_target_id++;
			}
		}
		//limit trajectory length
		for (auto &id_traj: targets) {
			auto &traj = id_traj.second;
			while (traj.size() >= TRAJECTORY_SIZE)
				traj.pop_front();
		}
		//purge old trajectories
		for (auto it = targets.cbegin(); it != targets.cend();) {
			const auto &id = it->first;
			const auto &traj = it->second;
			double timeout = TIMEOUT;
			if (id == target_focus)
				timeout = FOCUS_TIMEOUT;
			if (traj.size() == 0 || time - traj.back().time > timeout) {
				it = targets.erase(it);
				if (id == target_focus)
					target_focus = -1;
			} else {
				it++;
			}
		}
		publish_nearest();
	}
};

int main (int argc, char** argv) {
	std::string svm_filename = "/home/xlz/followme/src/followme/src/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
	float min_confidence = -1.5;
	float min_height = 1.3;
	float max_height = 2.3;
	float voxel_size = 0.06;
	Eigen::Matrix3f rgb_intrinsics_matrix;
	rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

	PointCloudT::Ptr cloud (new PointCloudT);
	bool new_cloud_available_flag = false;
	double new_cloud_timestamp = pcl::getTime();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind (&cloud_callback, _1, cloud,
			&new_cloud_available_flag, &new_cloud_timestamp);
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh("~");
	nh.param<std::string>("frame", target_frame, "/odom_combined");
	ros::Subscriber sub = nh.subscribe<PointCloudT> ("/head_mount_kinect/depth_registered/points", 1, f);
	ros::AsyncSpinner spinner(2);
	spinner.start();

	tf_ground_coeffs.resize(4);
	last_tf_ground_coeffs.resize(4);
	last_tf_ground_coeffs = tf_ground_coeffs;
	get_tf_ground_coeffs_once();
	Eigen::VectorXf ground_coeffs = tf_ground_coeffs;
	boost::thread groundThread(tf_ground_coeffs_callback);
	groundThread.detach();
	get_camera_transform();
	boost::thread cameraThread(camera_transform_callback);
	cameraThread.detach();

	// Wait for the first frame:
	while(!new_cloud_available_flag) 
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	new_cloud_available_flag = false;

	bool use_viewer;
	nh.param<bool>("viewer", use_viewer, false);
	pcl::visualization::PCLVisualizer *viewer = NULL;
	if (use_viewer) {
		viewer = new pcl::visualization::PCLVisualizer("Human detections");
		//set parallel projection
		/*vtkSmartPointer<vtkRendererCollection> rens = viewer.getRendererCollection();
		rens->InitTraversal();
		vtkRenderer *renderer = NULL;
		while ((renderer = rens->GetNextItem ()) != NULL) {
			vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera();
			cam->ParallelProjectionOn();
			cam->SetParallelScale(5);
			renderer->SetActiveCamera(cam);
		}*/
		//viewer.setCameraPosition(0, -600, 5, 0, 0, 5.01, 0, -1, 0);
		//viewer.setCameraPosition(0, -600, 0, 0, 0, 0.01, 0, -1, 0);
		viewer->setCameraFieldOfView(1./180*M_PI);
		//viewer.setCameraPosition(0, 0, -2, 0, -1, -0, 0);
		viewer->setCameraPosition(6, 0, -600, 6.01, 0, 0, 0, 0, 1);
	#if 0
		viewer.registerKeyboardCallback(keyboardEventOccurred, NULL);
	#endif
	}

	pcl::people::PersonClassifier<pcl::RGB> person_classifier;
	person_classifier.loadSVMFromFile(svm_filename);

	pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;
	people_detector.setVoxelSize(voxel_size);
	people_detector.setIntrinsics(rgb_intrinsics_matrix);
	people_detector.setClassifier(person_classifier);	
	people_detector.setHeightLimits(min_height, max_height);
	//people_detector.setSensorPortraitOrientation(true);	

	static unsigned count = 0;
	static double last = pcl::getTime ();
	bool draw_pointcloud;
	nh.param<bool>("pointcloud", draw_pointcloud, false);
	bool tracking = false;
	ROS_INFO("start");
	Tracker tracker(nh);
	PointCloudT::Ptr cloud2(cloud);
	while (ros::ok()/*!viewer.wasStopped()*/) {
		if (new_cloud_available_flag && cloud_mutex.try_lock ()) {
			camera_transform_mutex.lock();
			tf::StampedTransform cameraTransformNow(cameraTransform);
			camera_transform_mutex.unlock();
			new_cloud_available_flag = false;
			std::vector<pcl::people::PersonCluster<PointT> > clusters;
			people_detector.setInputCloud(cloud);
			if (tf_ground_coeffs_changed)
				getTFGroundCoeffs(ground_coeffs);
			people_detector.setGround(ground_coeffs);
			people_detector.compute(clusters);
			ground_coeffs = people_detector.getGround();

			if (use_viewer) {
				viewer->removeAllShapes();
				if (draw_pointcloud) {
					viewer->removeAllPointClouds();
					const auto &noground = people_detector.getNoGroundCloud();
					pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(noground);
					pcl_ros::transformPointCloud(*noground, *cloud2, cameraTransformNow);
					viewer->addPointCloud<PointT> (cloud2, rgb, "input_cloud");
				}
			}
			std::vector<Eigen::Vector2f> measurements;
			std::vector<float> confidences;

			for (auto &cluster: clusters) {
				Eigen::Vector2f pos;
				const auto &center = cluster.getTCenter();
				tf::Vector3 v(center[0], center[1], center[2]);
				v = cameraTransformNow * v;
				pos << v.getX(), v.getY();
				measurements.push_back(pos);
				confidences.push_back(cluster.getPersonConfidence());
			}
			tracker.update(new_cloud_timestamp, measurements, confidences);
			if (use_viewer) {
				tracker.draw(*viewer, new_cloud_timestamp);
				tf::Vector3 robotPos(0, 0, 0);
				robotPos = cameraTransformNow * robotPos;
				Eigen::Vector2f robotPos2;
				robotPos2 << robotPos.getX(), robotPos.getY();
				tracker.drawPosition(*viewer, robotPos2, -1);

				viewer->spinOnce();
			}

			if (++count == 30) {
				double now = pcl::getTime ();
				std::cerr << "Average framerate: " << double(count)/double(now - last) << " Hz" <<	std::endl;
				count = 0;
				last = now;
			}
			cloud_mutex.unlock ();
		}
	}

	delete viewer;
	ros::waitForShutdown();
	return 0;
}

