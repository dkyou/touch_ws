/*
发送力控指令给Touch手柄

根据手柄按键状态以及位姿进行力反馈操作

初始为自由状态（释放状态）
按下灰色按键进入力反馈状态（需在三轴0点附近按下，勿太靠近底座，避免刚进入即退出）
会根据手柄位置施加相应的力 使其保持在0点附近 超出一定空间范围自动退出力反馈
再次按下灰色按键退出力反馈状态
两个按键同时按下则进入锁死状态  手柄保持在当前位置  不可进行其它操作
再次双按则退出锁死状态  返回自由状态

庐州学子
2025.1.12
*/
#define PUBFORCE_H

#include "ros/ros.h"
#include "omni_msgs/OmniButtonEvent.h"
#include "geometry_msgs/PoseStamped.h"
#include "omni_msgs/OmniFeedback.h"
#include  <vector>

// 单轴输出的最大力 官网显示最大3.3N 测试可用1N
#define MAX_FORCE   1
#define RELEASE 0
#define FORCE_FEEDBACK  1
#define LOCK    2

//位姿与按键订阅者的回调函数
void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg);
void cb_button(const omni_msgs::OmniButtonEvent::ConstPtr &bt);
//释放手柄
void touch_release(void);
//在指定位置锁住touch手柄
void touch_lock(geometry_msgs::PoseStamped* pose);

// #include "geomagic_myhaptics/pubforce.h"

using namespace std;

//存储按键信息
bool flag_whitebutton=false;
bool flag_greybutton=false;
//Touch手柄状态：0  释放（自由状态） 1  力反馈状态  2   锁死状态
int touch_state=0;
ros::Publisher pub_force;
geometry_msgs::PoseStamped current_pose;

int main(int argc, char **argv)
{
    // 避免中文乱码
    setlocale(LC_ALL,"");

    // 初始化ROS节点
    ros::init(argc, argv, "pubforce");

    // 创建节点句柄
    ros::NodeHandle nh;

    ros::Subscriber sub_pose=nh.subscribe<geometry_msgs::PoseStamped>("/phantom/phantom/pose",3,cb_pose);
    ros::Subscriber sub_button = nh.subscribe<omni_msgs::OmniButtonEvent>("/phantom/phantom/button", 3, cb_button);
    pub_force=nh.advertise<omni_msgs::OmniFeedback>("/phantom/phantom/force_feedback",10);
    touch_state=RELEASE;

    ros::Rate r(30);

    //在while循环中执行任务
    while(ros::ok())
    {
        vector<uint8_t> lockv;
        omni_msgs::OmniFeedback fb;
        //接收最新的状态信息
        ros::spinOnce();

        // 双键按下 双按则锁死当前位置 再次双按则解锁
        if((flag_greybutton==true)&&(flag_whitebutton==true))
        {
            if(touch_state!=LOCK)
            {
                touch_lock(&current_pose);
                //双键按下后，全部置否，等待下一次按下,避免因长时间按住，重复操作
                flag_greybutton=false;
                flag_whitebutton=false;
            }
            else
            {
                touch_release();
                //双键按下后，全部置否，等待下一次按下,避免因长时间按住，重复操作
                flag_greybutton=false;
                flag_whitebutton=false;
            }
            continue;
        }
        //锁死状态下不进行力反馈操作
        if(touch_state==LOCK)
        {
            continue;
        }
        //按下灰色按键，启闭力反馈功能
        if(flag_greybutton==true)
        {
            if(touch_state!=FORCE_FEEDBACK)
            {
                touch_state=FORCE_FEEDBACK;
                ROS_INFO("手柄力反馈开启");
            }
            else
            {
                ROS_INFO("手柄力反馈关闭");
                touch_release();
            }
            //按下后，标志置否，等待下一次按下,避免因长时间按住，重复操作
            flag_greybutton=false;
        }
        //根据手柄位置施加相应力，使其归于三轴0点
        if(touch_state==FORCE_FEEDBACK)
        {
            // 超出运动范围则退出力反馈
            if(abs(current_pose.pose.position.x)>100||abs(current_pose.pose.position.y)>100||abs(current_pose.pose.position.z)>50)
            {
                ROS_INFO("超出运动范围，手柄力反馈关闭");
                touch_release();
                continue;
            }
            // 运动范围：x、y大概是正负两百，z正负50，具体可见echo /Geomagic/pose的信息
            // 这里取xy正负一百，z正负50
            // 根据位置 施加反方向的力即可
            fb.force.x=-((int)current_pose.pose.position.x/100.0)*MAX_FORCE;
            fb.force.y=-((int)current_pose.pose.position.y/100.0)*MAX_FORCE;
            fb.force.z=-((int)current_pose.pose.position.z/50.0)*MAX_FORCE;
            //三方向均解锁  需要解锁，否则不可实现力反馈
            lockv.push_back(0);
            lockv.push_back(0);
            lockv.push_back(0);
            // fb.lock=lockv;
            pub_force.publish(fb);
        }
        r.sleep();
    }

    return 0;
}


// 接收到订阅的消息后，会进入消息回调函数
// 注意，这里需要在前面加const，因后为const ptr
void cb_button(const omni_msgs::OmniButtonEvent::ConstPtr &bt)
{
    flag_whitebutton=true?bt->white_button==1:false;
    flag_greybutton=true?bt->grey_button==1:false;
    // ROS_INFO("按键状态改变：    灰色：%d，白色：%d",bt->grey_button,bt->white_button);
    // ROS_INFO("这时的位姿：      x: %.2f, y: %.2f, z: %.2f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    // ROS_INFO("这时的朝向：      w: %.2f, x: %.2f, y: %.2f, z: %.2f",current_pose.pose.orientation.w,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z);
}

void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
    //存储当前位置
    current_pose=*msg;
}

//在指定位置锁住touch手柄
void touch_lock(geometry_msgs::PoseStamped* pose)
{
    vector<uint8_t> lockv;
    omni_msgs::OmniFeedback fb;

    //三方向均锁死
    lockv.push_back(1);
    lockv.push_back(1);
    lockv.push_back(1);
    fb.position.x=pose->pose.position.x;
    fb.position.y=pose->pose.position.y;
    fb.position.z=pose->pose.position.z;
    // fb.lock=lockv;

    pub_force.publish(fb);

    touch_state=LOCK;

    ROS_INFO("手柄已锁住");
}


//释放Touch手柄 解锁、不输出力
void touch_release(void)
{
    vector<uint8_t> lockv;
    omni_msgs::OmniFeedback fb;
    lockv.push_back(0);
    lockv.push_back(0);
    lockv.push_back(0);
    // fb.lock=lockv;
    pub_force.publish(fb);

    touch_state=RELEASE;

    ROS_INFO("手柄已释放");
}
