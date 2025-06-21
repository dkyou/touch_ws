#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include "tf/transform_listener.h"
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

class Processor
{
public:
    Eigen::Matrix<double, 1, 6> pose_follow;

public:
    void Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

void Processor::Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double Tr[3] = {0, 0, 0}; // 定义位置xyz偏移量，进行如下变换
    Tr[0] = (msg->pose.position.x);
    Tr[1] = (msg->pose.position.z);
    Tr[2] = (msg->pose.position.y);
    
    tf::Quaternion qw1 = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    tf::Matrix3x3 R;
    tf::Quaternion qAC;
    double roll, pitch, yaw;
    R.setRotation(qw1); /*通过四元数计算得到旋转矩阵*/

    R.getRotation(qAC); //
    R.getRPY(roll, pitch, yaw);
    pitch=-pitch;
    yaw = -yaw;
    double pose_lead[6] = {Tr[0], Tr[1], Tr[2], roll, pitch, yaw};

    // 空间映射
    pose_follow << pose_lead[0], pose_lead[1], pose_lead[2], pose_lead[3], pose_lead[4], pose_lead[5];
    ROS_INFO("pose_follow:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",pose_follow(0),pose_follow(1),pose_follow(2),pose_follow(3),pose_follow(4),pose_follow(5));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_test");
    ros::NodeHandle nh;
    ros::NodeHandle node("~");

    Processor processor;

    ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/phantom/phantom/pose", 10, &Processor::Callback, &processor);
    
    // 逆解
    int num_samples;
    std::string chain_start, chain_end, urdf_param;
    double timeout;
    // const double error = 4e-4;
    const double error = 1e-6;

    node.param("chain_start", chain_start, std::string(""));
    node.param("chain_end", chain_end, std::string(""));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }

    node.param("timeout", timeout, 0.020);
    node.param("urdf_param", urdf_param, std::string("/robot_description"));

    if (num_samples < 1)
        num_samples = 1;


    // TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);  
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Distance);  

    KDL::Chain chain;
    bool valid = ik_solver.getKDLChain(chain);

    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }


    // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver based on kinematic chain

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    ROS_INFO ("Using %d joints",nj);


    KDL::Frame cartpos;
    KDL::JntArray joint_seed(nj);
    KDL::SetToZero(joint_seed);
    KDL::JntArray result(joint_seed);
    // 发布新的轨迹，发布到/robot_joint_states话题
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    sensor_msgs::JointState joint_states; // 消息声明

    joint_states.name.resize(6); // 定义6个运动节点
    joint_states.position.resize(6);

    joint_states.name[0] = "shoulder_joint";
    joint_states.name[1] = "upperArm_joint";
    joint_states.name[2] = "foreArm_joint";
    joint_states.name[3] = "wrist1_joint";
    joint_states.name[4] = "wrist2_joint";
    joint_states.name[5] = "wrist3_joint";

    double final_posatt[6] = {0, 0, 0, 0, 0, 0};
    ros::Rate loop_rate(500);

    //另外开启一个线程用于计算当前机械臂的关节角度
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        cartpos.p(0) = processor.pose_follow[0];
        cartpos.p(1) = processor.pose_follow[1];
        cartpos.p(2) = processor.pose_follow[2];
        cartpos.M = KDL::Rotation::RPY(processor.pose_follow[3], processor.pose_follow[4], processor.pose_follow[5]);
        // ROS_INFO("cartpos:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",cartpos.p(0),cartpos.p(1),cartpos.p(2),processor.pose_follow[3],processor.pose_follow[4],processor.pose_follow[5]);
        int rc = ik_solver.CartToJnt(joint_seed, cartpos, result);//joint_seed 5个关节的初始值，cartpos 6末端位姿，result 逆解
        if (rc >= 0)
        {
            ROS_INFO("Find the inverse solution");
            for (int i = 0; i < 6; i++)
            {
                joint_states.position[i] = result(i);
            
                joint_seed(i) = joint_states.position[i];
                final_posatt[i] = joint_states.position[i];
            }

        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                joint_states.position[i] = final_posatt[i];
            }
        }

        joint_states.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_states);
        ROS_INFO("joint_states:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",joint_states.position[0],joint_states.position[1],joint_states.position[2],joint_states.position[3],joint_states.position[4],joint_states.position[5]);
        // pub.publish(output);
        // ROS_INFO("pose_follow:[%.3f,%.3f,%.3f,%.3f,%.3f]",output.jointa1,output.jointa2,output.jointa3,output.jointa4,output.jointa5);
        /*loop_rate default 100 Hz */
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}