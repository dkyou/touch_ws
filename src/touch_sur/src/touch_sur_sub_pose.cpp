#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/MultiDOFJointState.h"
#include "std_msgs/Float64.h"
#include <Eigen/Core>

#include "ikfunc.h"
// #include "omni_msgs/OmniButtonEvent.h"
/*
订阅/phantom/pose话题，获取当前位置
订阅/joint_states_omni话题，获取当前B关节角度
发布/move_group/fake_controller_joint_states话题，发布当前位置和关节角度
*/

class touch_sur_sub
{

public:
    Eigen::Matrix<double,1,3> pose_follow;    
    std::array<double, 3> delta_l;//绳长变化
    std::array<double,2> beta_phi;//分别记录beta和phi
    void doTrans(const geometry_msgs::PoseStamped::ConstPtr &trans1);
    // void doButton(const geometry_msgs::PoseStamped::ConstPtr &trans1);
    // void doJoint(const sensor_msgs::JointState::ConstPtr &joint2);
};

void touch_sur_sub::doTrans(const geometry_msgs::PoseStamped::ConstPtr &trans1){
    //以圆上的点作为(x,y,z)路径点，求出逆解delta_l,并以追加的形式写入文件，初始值半径=5，圆心={0,0,5}，点的数量=100
    // writeCirclePointToFile();

    std::array<double, 2> beta_phi = {0.0, 0.0};
    std::array<double, 3> track = {0.0, 0.0,0.0};
    // if ()
    {
        pose_follow[0] = (trans1->pose.position.x);track[0] = pose_follow[0];
        pose_follow[1] = (trans1->pose.position.y);track[1] = pose_follow[1];
        pose_follow[2] = (trans1->pose.position.z);track[2] = pose_follow[2];
        // 将x,y,z写入文件
        writeArrayToFile("track.txt", track);
        beta_phi = inverseKinematics(pose_follow[0], pose_follow[1], pose_follow[2]);
        // 将beta和phi写入文件
        writeArrayToFile("beta_phi.txt", beta_phi);
    }
    

    
}

int main(int argc, char *argv[])
{
    touch_sur_sub trans1;
    touch_sur_sub Button;
    // touch_sur_sub joint2;

    //初始化一个节点:touch_updown
    ros::init(argc,argv,"touch_updown");
    ros::NodeHandle node; 
    //创建订阅者sub_trans1,订阅/phantom/pose话题，并打印信息
    ros::Subscriber sub_trans1 = node.subscribe<geometry_msgs::PoseStamped>("/phantom/pose",1,&touch_sur_sub::doTrans,&trans1);
    // ros::Subscriber sub2 = node.subscribe<omni_msgs::OmniButtonEvent>("/phantom/button",1,&touch_sur_sub::doTrans,&trans1);
    

    // 创建订阅者，订阅包含按钮状态的主题
    // ros::Subscriber sub = node.subscribe("/phantom/pose", 10, &touch_sur_sub::doButton, &Button);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}