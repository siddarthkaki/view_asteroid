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

	double omega = 0.25;
	Eigen::Vector3d omega0(0.0, 0.0, 0.25);
	ros::Time t0 = ros::Time::now();
	while (ros::ok()) {

		// Set asteroid's orientation
		float dt = (ros::Time::now() - t0).toSec();
		float roll = 0.0, pitch = -M_PI/2.0, yaw = omega*dt;
		Eigen::Quaterniond q1, q2, q;
		q1 = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
		q2 = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
		q = q2*q1;
		geometry_msgs::Quaternion quat;
		quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();

		// Asteroid pose
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

		loop_rate.sleep();
	}
	

	// ROS loop that starts callbacks/publishers
	ros::spin();

	return 0;
}