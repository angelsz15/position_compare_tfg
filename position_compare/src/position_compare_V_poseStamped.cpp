#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include <eigen3/Eigen/Dense>
#include "nav_msgs/Path.h"
#include <signal.h>


ros::Publisher path_pub;
nav_msgs::Path path;
geometry_msgs::PoseStamped ground_truth;
geometry_msgs::PoseStamped estimated_position;
bool pose1_initialized = false;
bool pose2_initialized = false;
double ARRE, ARPE;



Eigen::Matrix3d matrixLogarithm(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd angleAxis(R);
    double angle = angleAxis.angle();
    Eigen::Vector3d axis = angleAxis.axis();
    return angle * axis * axis.transpose();
}


void calculateRPE() {


    Eigen::Vector3d tpred(ground_truth.pose.position.x, ground_truth.pose.position.y, ground_truth.pose.position.z);
    Eigen::Vector3d tgt(estimated_position.pose.position.x, estimated_position.pose.position.y, estimated_position.pose.position.z);

        // Calcular el modulo de los vectores
        double norm_tpred = tpred.norm();
        double norm_tgt = tgt.norm();

        // Calcular el producto entre los vectores
        double dot_product = tpred.dot(tgt);

        // Valor total del RPE
        double RPE = std::acos(dot_product / (norm_tpred * norm_tgt));
        ROS_INFO("RPE actual: %f", RPE);
        ARPE += RPE;
        pose1_initialized = false;
        pose2_initialized = false;


}


void calculateRRE() {
    if (pose1_initialized && pose2_initialized) {
        // Compute Relative Rotation Error (RRE)
        Eigen::Quaterniond rpred(ground_truth.pose.orientation.w, ground_truth.pose.orientation.x, ground_truth.pose.orientation.y, ground_truth.pose.orientation.z);
        Eigen::Quaterniond rgt(estimated_position.pose.orientation.w, estimated_position.pose.orientation.x, estimated_position.pose.orientation.y, estimated_position.pose.orientation.z);
        
        Eigen::Matrix3d R_pred = rpred.toRotationMatrix();
        Eigen::Matrix3d R_gt = rgt.toRotationMatrix();
        
        // Calculo Matriz relativa positiva
        Eigen::Matrix3d R_relative = R_pred.transpose() * R_gt;

        // Calcular el logaritmo de la matriz de rotación relativa
        Eigen::Matrix3d log_R_relative = matrixLogarithm(R_relative);

        // Calcular la norma del logaritmo de la matriz de rotación relativa
        double RRE = log_R_relative.norm();

        ARRE += RRE;
        ROS_INFO("RRE actual: %f", RRE);
        calculateRPE();
    }
}

void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Pose recibida de /slam_out_pose:\n x: %f, y: %f, z: %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ground_truth = *msg; 
    pose1_initialized = true;
    calculateRRE();
}

void pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Pose recibida de /ze_vio/T_M_B:\n x: %f, y: %f, z: %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    estimated_position = *msg; 
    pose2_initialized = true;
    calculateRRE();
}

void printFINALValue() {
    ROS_INFO("Valor total de ARRE: %f", ARRE);
    ROS_INFO("Valor total de ARPE: %f", ARPE);
}

void sigintHandler(int sig) {
    printFINALValue();

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_subscriptor_node");
    ros::NodeHandle nh;

    ros::Subscriber pose1_sub = nh.subscribe("/slam_out_pose", 10, pose1Callback);
    ros::Subscriber pose2_sub = nh.subscribe("/ze_vio/T_M_B", 10, pose2Callback);
    path_pub = nh.advertise<nav_msgs::Path>("path_prueba", 10, true);
    signal(SIGINT, sigintHandler);

    ros::spin();

    return 0;
}

