#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include "tf/transform_listener.h"
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <mutex>

using namespace KDL;

class Processor
{
public:
    Eigen::Matrix<double, 1, 6> received_joint_positions;//存储接收到的关节角度（6个自由度）
    bool new_data_received;                 //dky：数据接收标志
    std::mutex data_mutex; // 添加互斥锁
public:
    void Callback(const sensor_msgs::JointState::ConstPtr &msg){
        std::lock_guard<std::mutex> lock(data_mutex);
        received_joint_positions = Eigen::Map<const Eigen::Matrix<double, 1, 6>>\
                                    (msg->position.data());
                                    // msg->position.size();
        new_data_received = true;
    }
    Processor():new_data_received(false){}//构造函数初始化标志
}; 
double roundToThreeDigits(double value) { return std::round(value * 1000.0) / 1000.0; }
/*
*接收/joint_states_omin的JontState消息，并提取出关节角度（6个自由度）
*/
// void Processor::Callback(const sensor_msgs::JointState::ConstPtr &msg)
// {
//     // std::lock_guard<std::mutex> lock(data_mutex);
//     // 空间映射
//     received_joint_positions << roundToThreeDigits(msg->position[0]) , \
//     roundToThreeDigits(msg->position[1]) , \
//     roundToThreeDigits(msg->position[2]) , \
//     roundToThreeDigits(msg->position[3]) , \
//     roundToThreeDigits(msg->position[4]) , \
//     roundToThreeDigits(msg->position[5]);
//     new_data_received = true;//标记接收到新数据
// }

void ikComputation(ros::NodeHandle &nh, ros::NodeHandle &node, Processor &processor)
{
    // 逆解
    std::string chain_start, chain_end, urdf_param;
    double timeout;
    const double error = 1e-6;

    node.param("chain_start", chain_start, std::string("chan_start"));
    node.param("chain_end", chain_end, std::string("chain_end"));
    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }
    node.param("timeout", timeout, 0.020);
    node.param("urdf_param", urdf_param, std::string("/robot_description"));

    //构造 TRAC-IK 求解器实例  自动解析 URDF 文件，提取基链接、末端链接和关节限制 初始化Chain
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Distance);  
    //获取 ik_solver 的 KDL chain链结构
    KDL::Chain chain;
    if (!ik_solver.getKDLChain(chain)) {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }
    //获取各关节的上下限
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits
    if (!ik_solver.getKDLLimits(ll, ul)){
        ROS_INFO("There were no valid KDL joint limits found");
        return;
    }
    //获取运动链chaim的关节数量
    unsigned int jntNum = chain.getNrOfJoints();//jntNum=6
    ROS_INFO("Using %d joints", jntNum);
        
    //正向运动学位置求解 递归算法计算机器人末端执行器的位姿 接收一个Chain对象的引用，初始化求解器
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    //初始化 IK 求解种子与结果变量
    bool kinematics_status;
    KDL::JntArray joint_seed(jntNum);//初始猜测值
    KDL::JntArray result(joint_seed);//存放 IK 结果

    for (uint j = 0; j < jntNum; j++) {
        //如果是第一次运行或者是数据未更新，使用中间位置
        if (!processor.new_data_received)
        {
            joint_seed(j) = (ll(j) + ul(j)) / 2.0;
        }else{
            joint_seed(j) = result(j);//否则使用上一次的结果
        }
    }

    //订阅/joint_states_omni关节状态，将数据传递给Callback ???/phantom/phantom/pose???
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states_omni", 3, &Processor::Callback, &processor);

    // 发布新的 JointState
    // 创建一个 JointState 消息, 发布到 /joint_states，可用于控制机械臂控制器 ???"/aubo_driver/controller_switch"???
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 3);
    
    sensor_msgs::JointState joint_states;

    joint_states.name.resize(6); // Define 6 joints
    joint_states.position.resize(6);
    joint_states.name = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"};
    joint_states.position = {0, 0, 0, 0, 0, 0};
    double final_posatt[6] = {0, 0, 0, 0, 0, 0};
    // ros::Rate loop_rate(500);
    ros::Rate loop_rate(100);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    double index = 0;

    //jointpositions: 当前关节角
    //cartpos: 当前末端位姿
    KDL::JntArray jointpositions = JntArray(jntNum);
    KDL::Frame cartpos;
    while (ros::ok())
    {
        //获取数据时加锁
        // std::lock_guard<std::mutex> lock(processor.data_mutex);
        for(unsigned int i=0; i<jntNum; i++){
            jointpositions(i) = (double)processor.received_joint_positions(i);
        }
        processor.new_data_received = false;

        kinematics_status = fk_solver.JntToCart(jointpositions, cartpos);

        Vector p = cartpos.p;   // Origin of the Frame
        Rotation M = cartpos.M; // Orientation of the Frame
        
        double roll, pitch, yaw;    
        M.GetRPY(roll, pitch, yaw);

        // 使用上次结果作为初始猜测值
        for (uint j = 0; j < jntNum; ++j) {
            joint_seed(j) = result(j); // 上次结果
        }

        int rc = ik_solver.CartToJnt(joint_seed, cartpos, result);
        if (rc < 0 ) {
            ROS_WARN("Inverse kinematics calculation failed");
            //使用上次的有效值
            for (int i = 0; i < 6; i++) {
                joint_states.position[i] = final_posatt[i];
            }
        }else{
            ROS_INFO("Find the inverse solution");
            for (int i = 0; i < 6; i++) {
                joint_states.position[i] = result(i); 
                final_posatt[i] = joint_states.position[i];
            }
        }
        
        joint_states.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_states);
        ROS_INFO("joint_states:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", \
            joint_states.position[0], joint_states.position[1], \
            joint_states.position[2], joint_states.position[3], \
            joint_states.position[4], joint_states.position[5]);

        // 在while循环中添加数据更新检查
        // if (!processor.new_data_received) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        // }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_test");
    ros::NodeHandle nh;
    ros::NodeHandle node("~");

    Processor processor;

    ikComputation(nh, node, processor);
    

    return 0;
}