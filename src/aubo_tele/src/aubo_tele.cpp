// #include <ros/ros.h>
// #include <Eigen/Core>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/JointState.h>
// #include <tf/transform_listener.h>
// #include "tf/transform_listener.h"
// #include <trac_ik/trac_ik.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>



// using namespace KDL;

// class Processor
// {
// public:
//     Eigen::Matrix<double, 1, 6> pose_follow;

// public:
//     void Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

// };
// double roundToThreeDigits(double value) { return std::round(value * 1000.0) / 1000.0; }

// void Processor::Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     double Tr[3] = {0, 0, 0}; // 定义位置xyz偏移量，进行如下变换
//     Tr[0] = (msg->pose.position.x * 18) ;
//     Tr[1] = (msg->pose.position.z  * 8);
//     Tr[2] = ((msg->pose.position.y  *20));
    
    
//     // Tr[0] = msg->pose.position.x;
//     // Tr[1] = msg->pose.position.y;
//     // Tr[2] = msg->pose.position.z;
//     tf::Quaternion qw1 = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
//     tf::Matrix3x3 R;
//     tf::Quaternion qAC;
//     double roll, pitch, yaw;
//     R.setRotation(qw1); /*通过四元数计算得到旋转矩阵*/

//     R.getRotation(qAC); //
//     R.getRPY(roll, pitch, yaw);
//     double pose_lead[6] = {Tr[0], Tr[1], Tr[2], roll, pitch, yaw};

//     // 空间映射
//     pose_follow << pose_lead[0], pose_lead[1], pose_lead[2], pose_lead[3], pose_lead[4], pose_lead[5];
// }
// void ikComputation(ros::NodeHandle &nh, ros::NodeHandle &node, Processor &processor)
// {
//     // 逆解
//     int num_samples;
//     std::string chain_start, chain_end, urdf_param;
//     double timeout;
//     const double error = 4e-4;

//     node.param("chain_start", chain_start, std::string(""));
//     node.param("chain_end", chain_end, std::string(""));

//     if (chain_start=="" || chain_end=="") {
//         ROS_FATAL("Missing chain info in launch file");
//         exit (-1);
//     }

//     node.param("timeout", timeout, 0.020);
//     node.param("urdf_param", urdf_param, std::string("/robot_description"));

//     if (num_samples < 1)
//         num_samples = 1;

//     // TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Distance);  
//     TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);  

//     KDL::Chain chain;
//     bool valid = ik_solver.getKDLChain(chain);

//     if (!valid) {
//         ROS_ERROR("There was no valid KDL chain found");
//         return;
//     }

//     // KDL::JntArray ll, ul; //lower joint limits, upper joint limits
//     // valid = ik_solver.getKDLLimits(ll, ul);  
//     // if (!valid)
//     //     ROS_INFO("There were no valid KDL joint limits found");

//     // ros::Subscriber sub1 = nh.subscribe<sensor_msgs::JointState>("/joint_states_omni", 3, &Processor::Callback, &processor);
//     ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/phantom/phantom/pose", 10, &Processor::Callback, &processor);

//     KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver based on kinematic chain
//     unsigned int nj = chain.getNrOfJoints();
//     ROS_INFO("Using %d joints", nj);



//     // KDL::Frame cartpos;
//     // KDL::JntArray joint_seed(nj);
//     // KDL::SetToZero(joint_seed);
//     // KDL::JntArray result(joint_seed);

//     KDL::Frame cartpos;    
//     KDL::JntArray jointpositions = JntArray(nj);
//     KDL::JntArray joint_seed(nj);
//     KDL::JntArray result(joint_seed);
//     // bool kinematics_status;

//     ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 3);
//     // ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/aubo_driver/controller_switch", 3);
//     sensor_msgs::JointState joint_states;

//     joint_states.name.resize(6); // Define 6 joints
//     joint_states.position.resize(6);
//     joint_states.name = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"};

//     ros::Rate loop_rate(500);
//     double final_posatt[6] = {0, 0, 0, 0, 0, 0};

//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     while (ros::ok())
//     {
//         // for(unsigned int i=0; i<nj; i++){
//         //     jointpositions(i) = (double)processor.pose_follow(i);
//         // }

//         // kinematics_status = fk_solver.JntToCart(jointpositions, cartpos);

//         Vector p = cartpos.p;   // Origin of the Frame
//         Rotation M = cartpos.M; // Orientation of the Frame
        
//         double roll, pitch, yaw;    
//         M.GetRPY(roll, pitch, yaw);
//         int rc = ik_solver.CartToJnt(joint_seed, cartpos, result);
//         if (rc < 0 ) {
//             printf("%s \n", "Error: could not calculate inverse kinematics :(");
//             for (int i = 0; i < 6; i++) {
//                 joint_states.position[i] = final_posatt[i];
//             }
//         }else{
//             ROS_INFO("Find the inverse solution");
//             for (int i = 0; i < 6; i++) {
//                 joint_states.position[i] = result(i); 
//                 final_posatt[i] = joint_states.position[i];
//             }
//         }


//         joint_states.header.stamp = ros::Time::now();
//         joint_state_pub.publish(joint_states);
//         ROS_INFO("joint_states:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", joint_states.position[0], joint_states.position[1], joint_states.position[2], joint_states.position[3], joint_states.position[4], joint_states.position[5]);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ik_test");
//     ros::NodeHandle nh;
//     ros::NodeHandle node("~");

//     Processor processor;

//     ikComputation(nh, node, processor);
    

//     return 0;
// }



// #include <ros/ros.h>
// #include <Eigen/Core>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/JointState.h>
// #include <tf/transform_listener.h>
// #include "tf/transform_listener.h"
// #include <trac_ik/trac_ik.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>



// using namespace KDL;

// class Processor
// {
// public:
//     Eigen::Matrix<double, 1, 6> pose_follow;

// public:
//     void Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

// };
// double roundToThreeDigits(double value) { return std::round(value * 1000.0) / 1000.0; }

// void Processor::Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     double Tr[3] = {0, 0, 0}; // 定义位置xyz偏移量，进行如下变换
//     Tr[0] = (msg->pose.position.x * 18) ;
//     Tr[1] = (msg->pose.position.z  * 8);
//     Tr[2] = ((msg->pose.position.y  *20));
    
    
//     // Tr[0] = msg->pose.position.x;
//     // Tr[1] = msg->pose.position.y;
//     // Tr[2] = msg->pose.position.z;
//     tf::Quaternion qw1 = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
//     tf::Matrix3x3 R;
//     tf::Quaternion qAC;
//     double roll, pitch, yaw;
//     R.setRotation(qw1); /*通过四元数计算得到旋转矩阵*/

//     R.getRotation(qAC); //
//     R.getRPY(roll, pitch, yaw);
//     double pose_lead[6] = {Tr[0], Tr[1], Tr[2], roll, pitch, yaw};

//     // 空间映射
//     pose_follow << pose_lead[0], pose_lead[1], pose_lead[2], pose_lead[3], pose_lead[4], pose_lead[5];
// }
// void ikComputation(ros::NodeHandle &nh, ros::NodeHandle &node, Processor &processor)
// {
//     // 逆解
//     int num_samples;
//     std::string chain_start, chain_end, urdf_param;
//     double timeout;
//     const double error = 1e-6;

//     node.param("chain_start", chain_start, std::string(""));
//     node.param("chain_end", chain_end, std::string(""));

//     if (chain_start=="" || chain_end=="") {
//         ROS_FATAL("Missing chain info in launch file");
//         exit (-1);
//     }

//     node.param("timeout", timeout, 0.020);
//     node.param("urdf_param", urdf_param, std::string("/robot_description"));

//     if (num_samples < 1)
//         num_samples = 1;

//     TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Distance);  
//     // TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);  

//     KDL::Chain chain;
//     bool valid = ik_solver.getKDLChain(chain);

//     if (!valid) {
//         ROS_ERROR("There was no valid KDL chain found");
//         return;
//     }

//     KDL::JntArray ll, ul; //lower joint limits, upper joint limits
//     valid = ik_solver.getKDLLimits(ll, ul);  
//     if (!valid)
//         ROS_INFO("There were no valid KDL joint limits found");

//     // ros::Subscriber sub1 = nh.subscribe<sensor_msgs::JointState>("/joint_states_omni", 3, &Processor::Callback, &processor);
//     ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/phantom/phantom/pose", 10, &Processor::Callback, &processor);

//     unsigned int nj = chain.getNrOfJoints();
//     ROS_INFO("Using %d joints", nj);

//     KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver based on kinematic chain

//     KDL::JntArray jointpositions = JntArray(nj);

//     KDL::Frame cartpos;    

//     bool kinematics_status;
//     KDL::JntArray joint_seed(nj);
//     KDL::JntArray result(joint_seed);

//     ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 3);
//     // ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/aubo_driver/controller_switch", 3);
//     sensor_msgs::JointState joint_states;

//     joint_states.name.resize(6); // Define 6 joints
//     joint_states.position.resize(6);
//     joint_states.name = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"};

//     double final_posatt[6] = {0, 0, 0, 0, 0, 0};
//     ros::Rate loop_rate(500);

//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     while (ros::ok())
//     {
//         for(unsigned int i=0; i<nj; i++){
//             jointpositions(i) = (double)processor.pose_follow(i);
//         }

//         kinematics_status = fk_solver.JntToCart(jointpositions, cartpos);

//         Vector p = cartpos.p;   // Origin of the Frame
//         Rotation M = cartpos.M; // Orientation of the Frame
        
//         double roll, pitch, yaw;    
//         M.GetRPY(roll, pitch, yaw);


//         for (uint j=0; j<joint_seed.data.size(); j++) {
//             joint_seed(j) = (ll(j) + ul(j)) / 2.0;
//         }

//         int rc = ik_solver.CartToJnt(joint_seed, cartpos, result);
//         if (rc < 0 ) {
//             printf("%s \n", "Error: could not calculate inverse kinematics :(");
//             for (int i = 0; i < 6; i++) {
//                 joint_states.position[i] = final_posatt[i];
//             }
//         }else{
//             ROS_INFO("Find the inverse solution");
//             for (int i = 0; i < 6; i++) {
//                 joint_states.position[i] = result(i); 
//                 final_posatt[i] = joint_states.position[i];
//             }
//         }


//         joint_states.header.stamp = ros::Time::now();
//         joint_state_pub.publish(joint_states);
//         ROS_INFO("joint_states:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", joint_states.position[0], joint_states.position[1], joint_states.position[2], joint_states.position[3], joint_states.position[4], joint_states.position[5]);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ik_test");
//     ros::NodeHandle nh;
//     ros::NodeHandle node("~");

//     Processor processor;

//     ikComputation(nh, node, processor);
    

//     return 0;
// }

#if 1
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



using namespace KDL;

class Processor
{
public:
    Eigen::Matrix<double, 1, 6> pose_follow;//存储接收到的关节角度（6个自由度）
    bool new_data_received;                 //dky：数据接收标志
public:
    void Callback(const sensor_msgs::JointState::ConstPtr &msg);
    Processor():new_data_received(false){}//构造函数初始化标志
}; 
double roundToThreeDigits(double value) { return std::round(value * 1000.0) / 1000.0; }
/*
*接收/joint_states_omin的JontState消息，并提取出关节角度（6个自由度）
*/
void Processor::Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // 空间映射
    pose_follow << roundToThreeDigits(msg->position[0]) , \
    roundToThreeDigits(msg->position[1]) , \
    roundToThreeDigits(msg->position[2]) , \
    roundToThreeDigits(msg->position[3]) , \
    roundToThreeDigits(msg->position[4]) , \
    roundToThreeDigits(msg->position[5]);
    // new_data_recevied = true;//标记接收到新数据
    // ROS_INFO("pose_follow:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",pose_follow(0),pose_follow(1),pose_follow(2),pose_follow(3),pose_follow(4),pose_follow(5));
}

void ikComputation(ros::NodeHandle &nh, ros::NodeHandle &node, Processor &processor)
{
    // 逆解
    int num_samples = 1;//初始化采样次数
    std::string chain_start, chain_end, urdf_param;
    double timeout;
    const double error = 1e-6;

    node.param("chain_start", chain_start, std::string(""));
    node.param("chain_end", chain_end, std::string(""));
    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }
    node.param("timeout", timeout, 0.020);
    node.param("urdf_param", urdf_param, std::string("/robot_description"));
    node.param("num_samples", num_samples, 1000);//存疑？
    if (num_samples < 1)
        num_samples = 1;

    //构造 TRAC-IK 求解器
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Distance);  
    // TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);  

    KDL::Chain chain;
    bool valid = ik_solver.getKDLChain(chain);//获取 KDL 链结构
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    KDL::JntArray ll, ul; //lower joint limits, upper joint limits
    valid = ik_solver.getKDLLimits(ll, ul);//获取各关节的上下限
    if (!valid)
        ROS_INFO("There were no valid KDL joint limits found");
    
    //订阅/joint_states_omni关节状态，将数据传递给Callback
    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::JointState>("/joint_states_omni", 3, &Processor::Callback, &processor);
    // ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/phantom/phantom/pose", 10, &Processor::Callback, &processor);

    unsigned int nj = chain.getNrOfJoints();//nj=6
    ROS_INFO("Using %d joints", nj);
    //初始化 FK 求解器与变量
    //fk_solver: 正运动学求解器
    //jointpositions: 当前关节角
    //cartpos: 当前末端位姿
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver based on kinematic chain

    KDL::JntArray jointpositions = JntArray(nj);

    KDL::Frame cartpos;    
    //初始化 IK 求解种子与结果变量
    bool kinematics_status;
    KDL::JntArray joint_seed(nj);//初始猜测值
    KDL::JntArray result(joint_seed);//存放 IK 结果

//新增开始
    // 初始化种子值为关节中间位置
    for (uint j = 0; j < nj; j++) {
        joint_seed(j) = (ll(j) + ul(j)) / 2.0;
        result(j) = joint_seed(j); // 初始化结果
    }
//新增结束


    //发布新的 JointState
    // 创建一个 JointState 消息并设置名称
    // 发布到 /joint_states，可用于控制机械臂控制器
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 3);
    // ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/aubo_driver/controller_switch", 3);
    sensor_msgs::JointState joint_states;

    joint_states.name.resize(6); // Define 6 joints
    joint_states.position.resize(6);
    joint_states.name = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"};
    joint_states.position = {0, 0, 0, 0, 0, 0};
    double final_posatt[6] = {0, 0, 0, 0, 0, 0};
    ros::Rate loop_rate(500);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    double index = 0;
    while (ros::ok())
    {
        for(unsigned int i=0; i<nj; i++){
            jointpositions(i) = (double)processor.pose_follow(i);
        }

        kinematics_status = fk_solver.JntToCart(jointpositions, cartpos);

        Vector p = cartpos.p;   // Origin of the Frame
        Rotation M = cartpos.M; // Orientation of the Frame
        
        double roll, pitch, yaw;    
        M.GetRPY(roll, pitch, yaw);


        // for (uint j=0; j<joint_seed.data.size(); j++) {
        //     joint_seed(j) = (ll(j) + ul(j)) / 2.0;
        // }
        // 使用上次结果作为初始猜测值
        for (uint j = 0; j < nj; ++j) {
            joint_seed(j) = result(j); // 上次结果
        }

        int rc = ik_solver.CartToJnt(joint_seed, cartpos, result);
        if (rc < 0 ) {
            printf("%s \n", "Error: could not calculate inverse kinematics :(");
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

        // joint_states:[-2.424, 0.115, 1.542, 1.441,-0.710,-1.139]
        // joint_states:[-2.424,-1.348,-1.542,-0.180,-0.710,-1.139]//稳定状态
        // joint_states:[-2.424,-1.348,-1.542,-0.180,-0.710,-1.139]
        // joint_states:[-2.424,-0.123, 0.687,-2.317, 0.710, 2.002]
        // joint_states:[-2.424,-1.348,-1.542,-0.180,-0.710,-1.139]
        // joint_states:[-2.424, 0.115, 1.542, 1.441,-0.710,-1.139]


        // double a[6]={0,0.5,0.5,0.5,0.5,0.5};
        // for(int j=0;j<6;j++){
        //     joint_states.position[j] = a[j];
        // }
        // joint_states.position[3]+=index;
        // index+=0.1;
        joint_states.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_states);
        ROS_INFO("joint_states:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", joint_states.position[0], joint_states.position[1], joint_states.position[2], joint_states.position[3], joint_states.position[4], joint_states.position[5]);

        ros::spinOnce();
        loop_rate.sleep();
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
#endif


// #include <ros/ros.h>
// #include <Eigen/Core>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/JointState.h>
// #include <tf/transform_listener.h>
// #include "tf/transform_listener.h"
// #include <trac_ik/trac_ik.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>

// using namespace KDL;

// class Processor
// {
// public:
//     Eigen::Matrix<double, 1, 6> pose_follow; // 存储接收到的关节角度（6个自由度）

// public:
//     void Callback(const sensor_msgs::JointState::ConstPtr &msg);
// };

// double roundToThreeDigits(double value) { return std::round(value * 1000.0) / 1000.0; }

// void Processor::Callback(const sensor_msgs::JointState::ConstPtr &msg)
// {
//     pose_follow << roundToThreeDigits(msg->position[0]),
//                    roundToThreeDigits(msg->position[1]),
//                    roundToThreeDigits(msg->position[2]),
//                    roundToThreeDigits(msg->position[3]),
//                    roundToThreeDigits(msg->position[4]),
//                    roundToThreeDigits(msg->position[5]);
// }

// void ikComputation(ros::NodeHandle &nh, ros::NodeHandle &node, Processor &processor)
// {
//     int num_samples = 0; // 已初始化
//     std::string chain_start, chain_end, urdf_param;
//     double timeout;
//     const double error = 1e-6;

//     node.param("chain_start", chain_start, std::string(""));
//     node.param("chain_end", chain_end, std::string(""));
//     if (chain_start == "" || chain_end == "") {
//         ROS_FATAL("Missing chain info in launch file");
//         exit(-1);
//     }
//     node.param("timeout", timeout, 0.020);
//     node.param("urdf_param", urdf_param, std::string("/robot_description"));

//     TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Distance);

//     KDL::Chain chain;
//     bool valid = ik_solver.getKDLChain(chain);
//     if (!valid) {
//         ROS_ERROR("There was no valid KDL chain found");
//         return;
//     }

//     KDL::JntArray ll, ul;
//     valid = ik_solver.getKDLLimits(ll, ul);
//     if (!valid)
//         ROS_INFO("There were no valid KDL joint limits found");

//     ros::Subscriber sub1 = nh.subscribe<sensor_msgs::JointState>("/joint_states_omni", 3, &Processor::Callback, &processor);

//     unsigned int nj = chain.getNrOfJoints();
//     ROS_INFO("Using %d joints", nj);

//     KDL::ChainFkSolverPos_recursive fk_solver(chain);
//     KDL::JntArray jointpositions(nj);
//     KDL::Frame cartpos;

//     KDL::JntArray joint_seed(nj);
//     KDL::JntArray result(nj); // 复用数组

//     ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 3);
//     sensor_msgs::JointState joint_states;
//     joint_states.name.resize(6);
//     joint_states.position.resize(6);
//     joint_states.name = {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"};

//     double final_posatt[6] = {0, 0, 0, 0, 0, 0};
//     ros::Rate loop_rate(10); // 提高频率到 10Hz

//     while (ros::ok())
//     {
//         for(unsigned int i=0; i<nj; i++) {
//             jointpositions(i) = (double)processor.pose_follow(i);
//         }

//         bool kinematics_status = fk_solver.JntToCart(jointpositions, cartpos);
//         // if (!kinematics_status) {
//         //     ROS_WARN("Forward kinematics failed, skipping iteration.");
//         //     continue;
//         // }

//         // 使用上次结果作为初始猜测
//         for (uint j = 0; j < nj; ++j) {
//             joint_seed(j) = result(j); // 上次结果
//         }

//         int rc = ik_solver.CartToJnt(joint_seed, cartpos, result);

//         if (rc < 0) {
//             ROS_WARN("IK failed with previous seed, retrying with mid-joint values...");
//             for (uint j = 0; j < nj; ++j) {
//                 joint_seed(j) = (ll(j) + ul(j)) / 2.0;
//             }
//             rc = ik_solver.CartToJnt(joint_seed, cartpos, result);
//         }

//         if (rc < 0) {
//             ROS_WARN("IK still failed after retry.");
//             for (int i = 0; i < 6; i++) {
//                 joint_states.position[i] = final_posatt[i];
//             }
//         } else {
//             for (int i = 0; i < 6; i++) {
//                 joint_states.position[i] = result(i);
//                 final_posatt[i] = result(i);
//             }
//             ROS_DEBUG("Find the inverse solution");
//         }

//         joint_states.header.stamp = ros::Time::now();
//         joint_state_pub.publish(joint_states);
//         ROS_DEBUG("joint_states:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
//                   joint_states.position[0], joint_states.position[1],
//                   joint_states.position[2], joint_states.position[3],
//                   joint_states.position[4], joint_states.position[5]);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ik_test");
//     ros::NodeHandle nh;
//     ros::NodeHandle node("~");

//     Processor processor;

//     ikComputation(nh, node, processor);

//     return 0;
// }