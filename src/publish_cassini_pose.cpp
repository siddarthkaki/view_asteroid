#include <ros/ros.h>
#include <Eigen/Dense>
#include "view_asteroid/helper.h"
#include "view_asteroid/rk4.h"
#include "view_asteroid/angvel.h"

// Global variable
rk4 attitude_integrator;

bool change_angvel(view_asteroid::angvel::Request &req,
                              view_asteroid::angvel::Request &res) {
    // std::cout << "service called: " << std::endl;
    ROS_INFO("Angular velocity set to: %f\t%f\t%f", req.vector.x, req.vector.y, req.vector.z);
    Eigen::Vector3d omega(req.vector.x, req.vector.y, req.vector.z);

    attitude_integrator.SetAngularVelocity(omega);

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "publish_asteroid_pose");
    ros::NodeHandle node("~");
    ROS_INFO("publish_asteroid_pose started!");
    const double rate = 100.0;
    ros::Rate loop_rate(rate);

    // Load initial conditions
    std::vector<double> initial_ang_vel, initial_quat_vector;
    double initial_quat_scalar;
    node.getParam("initial_ang_vel",     initial_ang_vel);
    node.getParam("initial_quat_vector", initial_quat_vector);
    node.getParam("initial_quat_scalar", initial_quat_scalar);

    // Load inertia matrix
    std::vector<double> J_mat;
    node.getParam("J_mat", J_mat);

    // Set subscribers to object and camera pose
    std::string obj_pose_topic, cam_pose_topic, obj_torque_topic;
    node.getParam("object_pose_topic",   obj_pose_topic);
    node.getParam("camera_pose_topic",   cam_pose_topic);
    node.getParam("object_torque_topic", obj_torque_topic);
    ros::Publisher asteroid_pose = 
        node.advertise<nav_msgs::Odometry>(obj_pose_topic, 1);
    ros::Publisher camera_pose = 
        node.advertise<geometry_msgs::Pose>(cam_pose_topic, 1);
    ros::Publisher object_torque = 
        node.advertise<geometry_msgs::Vector3>(obj_torque_topic, 1);

    // Initial conditions
    // double omega = 0.25;
    Eigen::Vector3d omega0(initial_ang_vel[0], initial_ang_vel[1], initial_ang_vel[2]);
    Eigen::Vector3d quat_vec(initial_quat_vector[0], initial_quat_vector[1], initial_quat_vector[2]);
    double quat_scalar = initial_quat_scalar;

    // Initialize rk4 integrator
    // Eigen::Matrix3d J = Eigen::Vector3d(0.637, 2.122, 2.235).asDiagonal();
    Eigen::Matrix3d J;
    J << J_mat[0], J_mat[1], J_mat[2],
         J_mat[3], J_mat[4], J_mat[5],
         J_mat[6], J_mat[7], J_mat[8];
    attitude_integrator = rk4(omega0, quat_scalar, quat_vec, J);

    // States
    Eigen::Vector3d omega_rk4;
    Eigen::Quaterniond q_rk4;

    //Initialize odometry message type
    nav_msgs::Odometry asteroid_odom;
    asteroid_odom.header.frame_id = "world";
    asteroid_odom.child_frame_id = "asteroid";

    // Initialize time
    ros::Time t0 = ros::Time::now();
    ros::Time t_prev = t0;
    ros::Time time_now;

    // Start service to change angular velocity of the object
    ros::ServiceServer update_angvel_srv = node.advertiseService("/UpdateAngVelObject", change_angvel);

    // Print initial conditions
    ROS_INFO("Initial angular velocity: %f\t%f\t%f", omega0[0], omega0[1], omega0[2]);
    ROS_INFO("Initial quaternion: qs: %f\tqv: %f\t%f\t%f", quat_scalar, quat_vec[0], quat_vec[1], quat_vec[2]);
    ROS_INFO("Inertia matrix: \n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f",
              J_mat[0], J_mat[1], J_mat[2], J_mat[3], J_mat[4], J_mat[5], J_mat[6], J_mat[7], J_mat[8]);

    // Update differential equations
    while (ros::ok()) {
        // Get times
        time_now = ros::Time::now();
        float dt = (time_now - t_prev).toSec();

        // Setting torque
        double A = 10, w = 0.1;
        ros::Duration t = time_now - t0;
        Eigen::Vector3d torque = A*Eigen::Vector3d(sin(w*t.toSec()),
                                                   sin(w*t.toSec() + 2.0*M_PI/3.0),
                                                   sin(w*t.toSec() + 4.0*M_PI/3.0));

        // Integrate RK4
        attitude_integrator.UpdateStates(dt, torque);
        attitude_integrator.GetStates(&omega_rk4, &q_rk4);

        // Set ROS structure for asteroid pose/twist
        geometry_msgs::Quaternion quat;
        quat.x = q_rk4.x(); quat.y = q_rk4.y();
        quat.z = q_rk4.z(); quat.w = q_rk4.w();
        asteroid_odom.pose.pose.position = helper::SetPoint(0.0, 0.0, 0.0);
        asteroid_odom.pose.pose.orientation = quat;
        asteroid_odom.twist.twist.linear = helper::SetVector3(0.0, 0.0, 0.0);
        asteroid_odom.twist.twist.angular = helper::SetVector3(omega_rk4[0], omega_rk4[1], omega_rk4[2]);
        asteroid_odom.header.stamp = ros::Time::now();

        // Camera pose
        geometry_msgs::Quaternion quat_cam;
        geometry_msgs::Point pos_cam = helper::SetPoint(-2.0, 0.0, 0.0);
        geometry_msgs::Pose pose_cam;
        quat_cam.x = 0.0; quat_cam.y = 0.0;
        quat_cam.z = 0.0; quat_cam.w = 1.0;
        pose_cam.position = pos_cam;
        pose_cam.orientation = quat_cam;

        // Publish poses
        asteroid_pose.publish(asteroid_odom);
        camera_pose.publish(pose_cam);
        object_torque.publish(helper::SetVector3(torque[0], torque[1], torque[2]));

        t_prev = time_now;

        ros::spinOnce();
        loop_rate.sleep();
    }
    

    // ROS loop that starts callbacks/publishers
    ros::spin();

    return 0;
}