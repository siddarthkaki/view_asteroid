#include <ros/ros.h>
#include <Eigen/Dense>
#include "view_asteroid/helper.h"
#include "view_asteroid/rk4.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "publish_asteroid_pose");
	ros::NodeHandle node("~");
	ROS_INFO("publish_asteroid_pose started!");
	const double rate = 100.0;
	ros::Rate loop_rate(rate);

	// Set subscribers to object and camera pose
	ros::Subscriber obj_subs, cam_subs;
	std::string obj_pose_topic, cam_pose_topic;
	node.getParam("object_pose_topic", obj_pose_topic);
	node.getParam("camera_pose_topic", cam_pose_topic);
	ros::Publisher asteroid_pose = 
		node.advertise<geometry_msgs::Pose>(obj_pose_topic, 1);
	ros::Publisher camera_pose = 
		node.advertise<geometry_msgs::Pose>(cam_pose_topic, 1);

	// Initial conditions
	// double omega = 0.25;
	Eigen::Vector3d omega0(0.15, 0.1, 0.05);
	Eigen::Vector3d quat_vec(0.0, -sin(M_PI/4), 0.0);
	double quat_scalar = cos(M_PI/4);

	// Initialize rk4 integrator
	Eigen::Matrix3d J = Eigen::Vector3d(0.637, 2.122, 2.235).asDiagonal();
	rk4 attitude_integrator(omega0, quat_scalar, quat_vec, J);

	// States
	Eigen::Vector3d omega_rk4;
	Eigen::Quaterniond q_rk4;

	// Initialize time
	ros::Time t_prev = ros::Time::now();
	ros::Time time_now;

	// Update differential equations
	while (ros::ok()) {

		// // Set asteroid's orientation
		// float dt = (ros::Time::now() - t0).toSec();
		// float roll = 0.0, pitch = -M_PI/2.0, yaw = omega*dt;
		// Eigen::Quaterniond q1, q2, q;
		// q1 = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
		// q2 = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
		// q = q2*q1;
		// geometry_msgs::Quaternion quat;
		// quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();

		// Integrate RK4
		time_now = ros::Time::now();
		float dt = (time_now - t_prev).toSec();
		attitude_integrator.UpdateStates(dt);
		attitude_integrator.GetStates(&omega_rk4, &q_rk4);

		// Set ROS structure for asteroid pose
		geometry_msgs::Quaternion quat;
		quat.x = q_rk4.x(); quat.y = q_rk4.y();
		quat.z = q_rk4.z(); quat.w = q_rk4.w();
		geometry_msgs::Point pos = helper::SetPoint(0.0, 0.0, 0.0);
		geometry_msgs::Pose pose;
		pose.position = pos;
		pose.orientation = quat;

		// Camera pose
		geometry_msgs::Quaternion quat_cam;
		geometry_msgs::Point pos_cam = helper::SetPoint(-2.0, 0.0, 0.0);
		geometry_msgs::Pose pose_cam;
		quat_cam.x = 0.0; quat_cam.y = 0.0;
		quat_cam.z = 0.0; quat_cam.w = 1.0;
		pose_cam.position = pos_cam;
		pose_cam.orientation = quat_cam;

		// Publish poses
		asteroid_pose.publish(pose);
		camera_pose.publish(pose_cam);

		t_prev = time_now;
		loop_rate.sleep();
	}
	

	// ROS loop that starts callbacks/publishers
	ros::spin();

	return 0;
}