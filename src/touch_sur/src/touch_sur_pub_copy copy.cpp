#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/MultiDOFJointState.h"
#include "std_msgs/Float64.h"
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <array>
#include <fstream>
// #include "omni_msgs/OmniButtonEvent.h"
/*
订阅/phantom/pose话题，获取当前位置
订阅/joint_states_omni话题，获取当前B关节角度
发布/move_group/fake_controller_joint_states话题，发布当前位置和关节角度
*/
// 绘制 3D 圆的函数
void plot_circle_3d(double radius, const std::vector<double>& center) {
    // 定义圆的参数范围
    int num_points = 100;  // 计算圆上的点的数量
    std::vector<double> theta(num_points), x(num_points), y(num_points), z(num_points, center[2]);
    
    // 计算圆上的点
    for (int i = 0; i < num_points; ++i) {
        theta[i] = 2 * M_PI * i / num_points;
        x[i] = center[0] + radius * cos(theta[i]);
        y[i] = center[1] + radius * sin(theta[i]);
    }

}
// Function for inverse kinematics calculation
void inverseKinematics(double L, double x, double y, double z, double &beta, double &phi) {
    // Calculate bending angle beta
    beta = 2 * std::atan2(std::sqrt(x * x + y * y), z);

    // Calculate rotation angle phi
    phi = std::atan2(y, x);
    if (phi < 0) {
        phi += 2 * M_PI;  // Adjust phi to be in the range [0, 2π]
    }
}



// Function to calculate rope length change from position
std::array<double, 3> calculateRopeLengthChangeFromPosition(double L, double r, double x, double y, double z) {
    // Inverse Kinematics - Calculate beta and phi
    double beta, phi;
    inverseKinematics(L, x, y, z, beta, phi);

    // Equivalent bending radius
    double R = L / beta;

    // Calculate length change for each of the 3 ropes
    std::array<double, 3> delta_l = {0.0, 0.0, 0.0};  // Store changes for 3 ropes
    for (int i = 0; i < 3; ++i) {
        // Bending radius for each rope
        double Ri = R - r * std::cos(phi - (2 * M_PI * i / 3));

        // Length change for this rope
        delta_l[i] = L - Ri * beta;
    }

    return delta_l;
}



class touch_sur_sub
{

public:
    Eigen::Matrix<double,1,3> pose_follow;    
    void doTrans(const geometry_msgs::PoseStamped::ConstPtr &trans1);
    // void doJoint(const sensor_msgs::JointState::ConstPtr &joint2);
};

void touch_sur_sub::doTrans(const geometry_msgs::PoseStamped::ConstPtr &trans1){
    double radius=10;
    const std::vector<double>& center = {0, 0, 5};
    int num_points = 100;  // 计算圆上的点的数量
    std::vector<double> theta(num_points), x(num_points), y(num_points), z(num_points, center[2]);
    std::array<double, 3> delta_l;
    std::string filename = "points.txt";
    // 计算圆上的点
    for (int i = 0; i < num_points; ++i) {
        theta[i] = 2 * M_PI * i / num_points;
        x[i] = center[0] + radius * cos(theta[i]);
        y[i] = center[1] + radius * sin(theta[i]);

        delta_l= calculateRopeLengthChangeFromPosition(10, 3, x[i], y[i], 5);
        ROS_INFO("delta_l[0] = %f,delta_l[1] = %f,delta_l[2] = %f\n",delta_l[0],delta_l[1],delta_l[2]);
    }
     // 打开文件用于写入
    std::ofstream out_file(filename);
    
    // 检查文件是否成功打开
    if (!out_file) {
        std::cerr << "无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    // 写入坐标到文件
    for (int i = 0; i < num_points; ++i) {
        out_file << x[i] << " " << y[i] << " " << z[i] << std::endl;
    }

    // 关闭文件
    out_file.close();
    std::cout<<"文件 " << filename << " 写入成功！" << std::endl;
    
    // ROS_INFO("trans1->pose.position.z = %f",trans1->pose.position.z);
    // pose_follow[0] = 0.1*(trans1->pose.position.x);
    // pose_follow[1] = 0.1*(trans1->pose.position.y);
    // pose_follow[2] = 0.1*(trans1->pose.position.z);
    // ROS_INFO("trans1->pose.position.x = %f,trans1->pose.position.y = %f,trans1->pose.position.z = %f\n",
    // trans1->pose.position.x,trans1->pose.position.y,trans1->pose.position.z);
    // ROS_INFO("delta_l[0] = %f,delta_l[1] = %f,delta_l[2] = %f\n",delta_l[0],delta_l[1],delta_l[2]);


    
}

// void touch_sur_sub::doJoint(const sensor_msgs::JointState::ConstPtr &joint2){
//     ROS_INFO("joint2 = %f",joint2->position[5] + 3.14);
//     // pose_follow[1] = serprecision(3);
//     pose_follow[1] = joint2->position[5] + 3.14;
//     // 角度=180°×弧度÷π

// }

int main(int argc, char *argv[])
{
    touch_sur_sub trans1;
    // touch_sur_sub joint2;

    //初始化一个节点:touch_updown
    ros::init(argc,argv,"touch_updown");
    ros::NodeHandle node; 
    //创建订阅者sub_trans1,订阅/phantom/pose话题，并打印信息
    ros::Subscriber sub_trans1 = node.subscribe<geometry_msgs::PoseStamped>("/phantom/pose",1,&touch_sur_sub::doTrans,&trans1);
    // ros::Subscriber sub2 = node.subscribe<omni_msgs::OmniButtonEvent>("/phantom/button",1,&touch_sur_sub::doTrans,&trans1);
    // ros::Subscriber sub_joint2 = node.subscribe<sensor_msgs::JointState>("/joint_states_omni",1,&touch_sur_sub::doJoint,&joint2);
    //创建一个pub发布对象，
    //用于发布sensor_msgs::JointState类型的消息到/move_group/fake_controller_joint_states话题
    // ros::Publisher pub = node.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 100);
    

    
    // sensor_msgs::JointState joint_states;//消息声明
    // joint_states.position.resize(2);
    // joint_states.name.resize(2); 
    // joint_states.name[0] = "trans1";
    // joint_states.name[1] = "trans2";

    // joint_states.header.stamp = ros::Time::now();

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        // joint_states.position[0] = trans1.pose_follow[0];
        // joint_states.position[1] = joint2.pose_follow[1];

        // pub.publish(joint_states);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}