#include <ros/ros.h>
#include <string>
#include <cstring>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <sstream>
#include <pthread.h>
#include <assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"

#define BT_EULER_DEFAULT_ZYX

// #define OMNI_STATE_CALLBACK_DEBUG
#define DEVICE_NAME "Default Device"
// #define DEVICE_NAME "Default PHANTOM"
// #define DEVICE_NAME "R"
int calibrationStyle;
bool enable_debug_print = true;
int print_counter = 0;

struct OmniState {
  // hduVector3Dd表示一个三维向量的类 是一个包含 3 个 double 类型元素 的向量
  hduVector3Dd position;  //3x1 vector of position 从变换矩阵提取到的位置
  hduVector3Dd positionOrigin;  //3x1 vector of position origin 直接从传感器获取的位置
  hduVector3Dd lock_pos;

  hduVector3Dd velocity;  //3x1 vector of velocity 由坐标位置计算得到的速度
  hduVector3Dd velocityOrigin;  //3x1 vector of velocity origin 直接从传感器得到的位置
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;  // 输入的速度，可能是从外部设备接收到的速度值
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;

  hduQuaternion rotation;//从变换矩阵中得到四元数

  hduVector3Dd joints;       //double joints[3]//前三个关节的角度
  hduVector3Dd gimbal_angles;//double joints[3]//后三个关节的角度
  double thetas[7];//存储3+3个关节角度
  double thetasOrigin[7];//存储3+3个关节角度 数据来源于joints + gimbal_angles

  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]//TODO
  hduVector3Dd forceGetOrigin;//TODO

  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  
  double units_ratio;
  OmniState() : position(), positionOrigin(), velocity(), 
                rotation(), joints(), gimbal_angles(), 
                force(), lock(false), lock_pos(), 
                close_gripper(false), units_ratio(1.0) {
    inp_vel1 = hduVector3Dd();
    inp_vel2 = hduVector3Dd();
    inp_vel3 = hduVector3Dd();
    out_vel1 = hduVector3Dd();
    out_vel2 = hduVector3Dd();
    out_vel3 = hduVector3Dd();
    pos_hist1 = hduVector3Dd();
    pos_hist2 = hduVector3Dd();
    for (unsigned int i = 0; i < 7; i++) thetas[i] = 0.0f;
    buttons[0] = buttons[1] = 0;
    buttons_prev[0] = buttons_prev[1] = 0;
  }
};
/* class PhantomRos starts here                                               *      
 *****************************************************************************/
class PhantomROS {
private:
  ros::NodeHandle nh;
  ros::Publisher omni_state_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher button_publisher;
  ros::Publisher joint_publisher;
  ros::Subscriber haptic_sub;
  
  std::string omni_name, ref_frame, units;
  // std::unique_ptr<OmniState> OmniStatePtr;
  struct OmniState *OmniStatePtr;

  bool initialized;
  public:
  PhantomROS() : 
              initialized(false),OmniStatePtr(nullptr){
    // OmniStatePtr = new OmniState();//创建OmniState实例
  }
  ~PhantomROS() {
    cleanup();
    // if(OmniStatePtr) delete OmniStatePtr;
  }
  // void init(OmniState *s) {
  bool init() {
    if(initialized){
      ROS_WARN("PhantomROS already initialized!");
      return true;
    }
    
    ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));
    
    
    ROS_INFO("===============Initializing Phantom ROS node for [%s]", omni_name.c_str());
    ROS_INFO("===============Using reference frame [%s]", ref_frame.c_str());
    ROS_INFO("===============Using units [%s]", units.c_str());
    //创建 phantom/button 按钮状态话题，，并发布 OmniButtonEvent 消息，队列长度为 100。
    std::ostringstream stream1;
    stream1 << omni_name << "/button";
    std::string button_topic = std::string(stream1.str());
    button_publisher = nh.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100);

    //创建 phantom/state 状态话题，并发布 OmniState 消息，队列长度为 1。
    std::ostringstream stream2;
    stream2 << omni_name << "/state";
    std::string state_topic_name = std::string(stream2.str());
    omni_state_publisher = nh.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1);

    // 订阅 omni_name/force_feedback 话题，回调函数为 force_callback，队列长度为 1。
    std::ostringstream stream3;
    stream3 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = nh.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

    // 创建 omni_name/pose 姿态话题，并发布 PoseStamped 消息，队列长度为 1。
    std::ostringstream stream4;
    stream4 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1);

    // 创建 /joint_states_omni 关节状态话题，并发布 JointState 消息，队列长度为 1。
    std::ostringstream stream5;
    stream5 << "/joint_states_omni";
    // stream5 << omni_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = nh.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);
    
    /*-----------------------初始化Omni状态-------------------------*/
    OmniStatePtr = new OmniState();//创建OmniState实例
    if (!OmniStatePtr) {
            ROS_FATAL("Failed to allocate OmniState memory");
            return false;
    }

    if (!units.compare("mm")) OmniStatePtr->units_ratio = 1.0;
    else if (!units.compare("cm")) OmniStatePtr->units_ratio = 10.0;
    else if (!units.compare("dm")) OmniStatePtr->units_ratio = 100.0;
    else if (!units.compare("m")) OmniStatePtr->units_ratio = 1000.0;
    else{
      OmniStatePtr->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHANTOM position given in [%s], ratio [%.1f]", units.c_str(), OmniStatePtr->units_ratio);
    // 初始化按钮状态：
    OmniStatePtr->buttons[0] = 0;
    OmniStatePtr->buttons[1] = 0;
    OmniStatePtr->buttons_prev[0] = 0;
    OmniStatePtr->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    OmniStatePtr->velocity = zeros;
    OmniStatePtr->velocityOrigin = zeros;
    OmniStatePtr->inp_vel1 = zeros; 
    OmniStatePtr->inp_vel2 = zeros; 
    OmniStatePtr->inp_vel3 = zeros; 
    OmniStatePtr->out_vel1 = zeros; 
    OmniStatePtr->out_vel2 = zeros; 
    OmniStatePtr->out_vel3 = zeros; 
    OmniStatePtr->pos_hist1 = zeros;
    OmniStatePtr->pos_hist2 = zeros;
    OmniStatePtr->lock = false;
    OmniStatePtr->close_gripper = false;
    OmniStatePtr->lock_pos = zeros;

    initialized = true;
    return true;
  }
void cleanup() {
    if (!initialized) return;
    // 关闭发布器和订阅器
    omni_state_publisher.shutdown();
    pose_publisher.shutdown();
    button_publisher.shutdown();
    joint_publisher.shutdown();
    haptic_sub.shutdown();
    // 释放OmniStatePtr内存
    if (OmniStatePtr) {
        delete OmniStatePtr;
        OmniStatePtr = nullptr;
    }
    ROS_INFO("PhantomROS resources cleaned up");
    initialized = false;
}
OmniState* getOmniStatePtr() const{
  return OmniStatePtr;
}



  /*******************************************************************************
   发布OmniStateMsg
   发布PoseStampedMsg
   *******************************************************************************/
  void publish_omni_state_and_pose_stampe() {
    if (!initialized || !OmniStatePtr) return;

    const ros::Time current_time = ros::Time::now();//统一时间戳

    omni_msgs::OmniState OmniStateMsg;
    // 设置锁定状态和夹爪状态
    OmniStateMsg.locked = OmniStatePtr->lock;
    OmniStateMsg.close_gripper = OmniStatePtr->close_gripper;
    // Position
    OmniStateMsg.pose.position.x = OmniStatePtr->position[0];
    OmniStateMsg.pose.position.y = OmniStatePtr->position[1];
    OmniStateMsg.pose.position.z = OmniStatePtr->position[2];
    // Orientation
    OmniStateMsg.pose.orientation.x = OmniStatePtr->rotation.v()[0];
    OmniStateMsg.pose.orientation.y = OmniStatePtr->rotation.v()[1];
    OmniStateMsg.pose.orientation.z = OmniStatePtr->rotation.v()[2];
    OmniStateMsg.pose.orientation.w = OmniStatePtr->rotation.s();
    // Velocity
    OmniStateMsg.velocity.x = OmniStatePtr->velocity[0];
    OmniStateMsg.velocity.y = OmniStatePtr->velocity[1];
    OmniStateMsg.velocity.z = OmniStatePtr->velocity[2];
    // TODO: Append Current to the state msg
    OmniStateMsg.header.stamp = current_time;
    //发布OmniState msg
    omni_state_publisher.publish(OmniStateMsg);

  /*******************************************************************************
     发布PoseStampedMsg
    *******************************************************************************/
    geometry_msgs::PoseStamped PoseStampedMsg;
    PoseStampedMsg.header.stamp = current_time;//统一时间戳
    PoseStampedMsg.header.frame_id = ref_frame;
    //设置位置信息并转换单位
    PoseStampedMsg.pose = OmniStateMsg.pose;
    PoseStampedMsg.pose.position.x /= 1000.0;
    PoseStampedMsg.pose.position.y /= 1000.0;
    PoseStampedMsg.pose.position.z /= 1000.0;

    pose_publisher.publish(PoseStampedMsg);

  }
/*******************************************************************************
   发布OmniButtonEventMsg
  *******************************************************************************/
void publish_omni_button_event() {
  if (!initialized || !OmniStatePtr) return;
  //只在按钮状态变化时发布
  bool grey_button_rising = (OmniStatePtr->buttons[0] == 1 && OmniStatePtr->buttons_prev[0] == 0);
  bool white_button_rising = (OmniStatePtr->buttons[1] == 1 && OmniStatePtr->buttons_prev[1] == 0);
  
  if (grey_button_rising || white_button_rising)
  {
      omni_msgs::OmniButtonEvent OmniButtonEventMsg;
      OmniButtonEventMsg.grey_button = OmniStatePtr->buttons[0];
      OmniButtonEventMsg.white_button = OmniStatePtr->buttons[1];
      button_publisher.publish(OmniButtonEventMsg);
      // 处理按钮事件
      if (grey_button_rising) {
          OmniStatePtr->close_gripper = !OmniStatePtr->close_gripper;
          ROS_INFO("Gripper state changed to: %s", 
                       OmniStatePtr->close_gripper ? "CLOSED" : "OPEN");
      }
      if (white_button_rising) {
          OmniStatePtr->lock = !OmniStatePtr->lock;
          ROS_INFO("Lock state changed to: %s", 
                       OmniStatePtr->lock ? "LOCKED" : "UNLOCKED");
      }
      // 更新按钮状态历史
      OmniStatePtr->buttons_prev[0] = OmniStatePtr->buttons[0];
      OmniStatePtr->buttons_prev[1] = OmniStatePtr->buttons[1];
  }
}
/*******************************************************************************
   发布JointStateMsg
  *******************************************************************************/
void publish_omni_joint_state(){
  if (!initialized || !OmniStatePtr) return;
  const ros::Time current_time = ros::Time::now();
  sensor_msgs::JointState JointStateMsg;

  JointStateMsg.header.stamp = current_time;
  JointStateMsg.name.resize(6);
  JointStateMsg.position.resize(6);
  //waist 腰部
  JointStateMsg.name[0] = "waist";
  JointStateMsg.position[0] = -OmniStatePtr->thetas[1];
  //shoulder 肩；肩部
  JointStateMsg.name[1] = "shoulder";
  JointStateMsg.position[1] = OmniStatePtr->thetas[2];
  //elbow肘；肘部
  JointStateMsg.name[2] = "elbow";
  JointStateMsg.position[2] = OmniStatePtr->thetas[3];
  //yaw 偏航
  JointStateMsg.name[3] = "yaw";
  JointStateMsg.position[3] = -OmniStatePtr->thetas[4] + M_PI;
  //pitch 俯仰；（飞机等的）上下摆动
  JointStateMsg.name[4] = "pitch";
  JointStateMsg.position[4] = -OmniStatePtr->thetas[5] - 3*M_PI/4;
  //roll 翻滚；侧滚
  JointStateMsg.name[5] = "roll";
  JointStateMsg.position[5] = -OmniStatePtr->thetas[6] - M_PI;

  joint_publisher.publish(JointStateMsg);
}
/*******************************************************************************
   发布OmniFeedbackMsg
  *******************************************************************************/  
void subscribe_omni_feedback() {
  if (!initialized || !OmniStatePtr) return;
  omni_msgs::OmniFeedback OmniFeedbackMsg;
  // 获取位置信息：
  OmniFeedbackMsg.position.x = OmniStatePtr->position[0];
  OmniFeedbackMsg.position.y = OmniStatePtr->position[1];
  OmniFeedbackMsg.position.z = OmniStatePtr->position[2];
  // Orientation
  // 获取姿态信息
}
/*******************************************************************************
 ROS node callback.
 *******************************************************************************/
void force_callback(const omni_msgs::OmniFeedbackConstPtr& OmniFeedbackMsg) {
  ////////////////////Some people might not like this extra damping, but it
  ////////////////////helps to stabilize the overall force feedback. It isn't
  ////////////////////like we are getting direct impedance matching from the
  ////////////////////omni anyway
  //力反馈处理：
  // 从 omnifeed 中获取力值，并减去一个与速度成正比的小量（0.001 * 速度），以增加稳定性。
  //更新 state->force 数组中的三个分量（x, y, z）。
  if (!initialized || !OmniStatePtr) return;

  OmniStatePtr->force[0] = OmniFeedbackMsg->force.x - 0.001 * OmniStatePtr->velocity[0];
  OmniStatePtr->force[1] = OmniFeedbackMsg->force.y - 0.001 * OmniStatePtr->velocity[1];
  OmniStatePtr->force[2] = OmniFeedbackMsg->force.z - 0.001 * OmniStatePtr->velocity[2];
  //位置锁定:
  //直接将 omnifeed 中的位置值赋给 state->lock_pos 数组中的三个分量（x, y, z）。
  //位置同步：确保 state 对象中的位置信息与 omnifeed 提供的位置信息保持一致，实现位置的实时更新。
  OmniStatePtr->lock_pos[0] = OmniFeedbackMsg->position.x;
  OmniStatePtr->lock_pos[1] = OmniFeedbackMsg->position.y;
  OmniStatePtr->lock_pos[2] = OmniFeedbackMsg->position.z;
}
void print_omni_state() {
    if (!enable_debug_print) return;
    
    if (++print_counter % 10 != 0) return;  // 控制输出频率

    ROS_INFO(">--------%sRunning--------<", omni_name.c_str());
    ROS_INFO("ref_frame = %s\tunits = %s", ref_frame.c_str(), units.c_str());

    for (int i = 0; i < 3; ++i)
        ROS_INFO("position[%d] = %f", i, OmniStatePtr->position[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("positionOrigin[%d] = %f", i, OmniStatePtr->positionOrigin[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("velocity[%d] = %f\t", i, OmniStatePtr->velocity[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("velocityOrigin[%d] = %f\t", i, OmniStatePtr->velocityOrigin[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("joints[%d] = %f\t", i, OmniStatePtr->joints[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("gimbal_angles[%d] = %f\t", i, OmniStatePtr->gimbal_angles[i]);
    for (int i = 1; i < 7; ++i)
        ROS_INFO("thetas[%d] = %f", i, OmniStatePtr->thetas[i]);
    for (int i = 1; i < 7; ++i)
        ROS_INFO("thetasOrigin[%d] = %f", i, OmniStatePtr->thetasOrigin[i]);
    
    for (int i = 0; i < 3; ++i)
        ROS_INFO("force[%d] = %f", i, OmniStatePtr->force[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("forceGetOrigin[%d] = %f", i, OmniStatePtr->forceGetOrigin[i]);
    for (int i = 0; i < 3; ++i)
        ROS_INFO("lock_pos[%d] = %f", i, OmniStatePtr->lock_pos[i]);
    for (int i = 0; i < 4; ++i)
        ROS_INFO("OmniStatePtr->rotation[%d] = %f\t", i, OmniStatePtr->rotation[i]);
    std::cout << "Rotation quaternion: " << OmniStatePtr->rotation << "\n";
    for (int i = 0; i < 2; ++i)
        ROS_INFO("buttons[%d] = %d", i, OmniStatePtr->buttons[i]);
    ROS_INFO("lock state = %d",OmniStatePtr->lock);
    ROS_INFO("close_gripper = %d",OmniStatePtr->close_gripper);
    
}
};
/* class PhantomRos end                                                       *      
 *****************************************************************************/
void debugOmniState(OmniState* omni_state_ptr, const hduMatrix& transform,
                    const hduVector3Dd& gimbal_angles,
                    const hduMatrix& rotation,
                    const hduMatrix& rotation_offset) {
    std::cout << "Address of transform: " << &transform << std::endl;
    std::cout << "Address of first element: " << &transform[0][0] << std::endl;

    std::cout << "Transform Matrix:" << std::endl;
    std::cout << transform << std::endl;

    std::cout << "positionOrigin:" << std::endl;
    std::cout << omni_state_ptr->positionOrigin << std::endl;

    std::cout << "Joint Angles:" << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << "Joint " << i << ": " << omni_state_ptr->joints[i] << std::endl;
    }

    std::cout << "Gimbal Angles: ";
    for (int i = 0; i < 3; ++i) {
        std::cout << gimbal_angles[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Rotation Matrix:" << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << std::left << std::setw(12) << rotation[i][j];
        }
        std::cout << std::endl;
    }

    std::cout << "rotation_offset Matrix:" << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout << std::left << std::setw(12) << rotation_offset[i][j];
        }
        std::cout << std::endl;
    }

    std::cout << "hduQuaternion Matrix:" << std::endl;
    std::cout << hduQuaternion(rotation_offset * rotation) << std::endl;

    std::cout << std::endl;

    std::cout << "velocityOrigin : ";
    for (int i = 0; i < 3; ++i) {
        std::cout << omni_state_ptr->velocityOrigin[i] << " ";
    }
    std::cout << std::endl;
}
/*用于处理 Omni 设备的状态更新,每次设备状态变化时被调用，负责获取设备的各种状态信息并进行相应的处理*/
HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  // OmniState *omni_state = static_cast<OmniState *>(pUserData);
  PhantomROS* phantom_ros = static_cast<PhantomROS *>(pUserData);
  // OmniState* omni_state_ptr = phantom_ros->OmniStatePtr;
  // OmniState* omni_state_ptr = phantom_ros->getOmniStatePtr();
  OmniState* omni_state_ptr = phantom_ros->getOmniStatePtr();


  //检查设备是否需要重新校准，如果需要则使用指定的calibrationstyle方式校准
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
  //开始处理当前帧
  hdBeginFrame(hdGetCurrentDevice());

  int nButtons = 0;//位掩码的形式
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state_ptr->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;//0b0001 & 0b0011 = 1
  omni_state_ptr->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;//0b0010 & 0b0011 = 1
  if (omni_state_ptr->buttons[0] == 1){
    omni_state_ptr->lock = true;
  }
  if (omni_state_ptr->buttons[1] == 1){
    omni_state_ptr->lock = false;
  }
  /*
  * HD_CURRENT_POSITION
  *    +Y (向上)
  *     |
  *     |
  *     |
  *     O-----> +X (向右)
  *    /
  *   /
  *  / +Z (朝向用户)
  */
  hduMatrix transform;
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);//获取当前设备末端的位置和姿态（4x4变换矩阵）
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state_ptr->joints);//获取一组关节角度值 For Touch devices: Turet Left +, Thigh Up +,Shin Up +
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state_ptr->gimbal_angles);//获取后三个关节角度
  hdGetDoublev(HD_CURRENT_VELOCITY,omni_state_ptr->velocityOrigin);//[0]左右 [1]前后 [2]上下
  hdGetDoublev(HD_CURRENT_POSITION,omni_state_ptr->positionOrigin);

  // Notice that we are inverting the Z-position value and changing Y <---> Z
  //1. 反转 Z 轴位置值 相当于绕 X 轴旋转 180°（或绕 Y 轴旋转 180°），使 Z 轴方向与原方向相反。
  // 2. 交换 Y 轴和 Z 轴 相当于绕 X 轴旋转 90°，使原 Y 轴方向变为新 Z 轴方向，原 Z 轴方向变为新 Y 轴方向。
  // 总的：绕 X 轴旋转 90° + 关于 XY 平面的镜像（或类似操作），具体取决于坐标系的初始手性
  // Position
  omni_state_ptr->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  omni_state_ptr->position /= omni_state_ptr->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  //绕 Z 轴顺时针旋转 90 度的变换 
  //将原坐标系的 X 轴映射到新 Y 轴正方向
  //将原坐标系的 Y 轴映射到新 X 轴负方向
  //Z 轴方向保持不变
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  // rotation_offset.operator*(rotation)
  omni_state_ptr->rotation = hduQuaternion(rotation_offset * rotation);

  //调试rotation_offset 和 rotation
  // Velocity estimation
  // 利用当前位置和前两次历史位置计算瞬时速度
  hduVector3Dd vel_buff(0, 0, 0);
  // 使用二阶后向差分法估算速度；
  vel_buff = (omni_state_ptr->position * 3 - 4 * omni_state_ptr->pos_hist1
      + omni_state_ptr->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  //应用低通滤波器平滑速度输出（截止频率20Hz）；
  omni_state_ptr->velocity = (.2196 * (vel_buff + omni_state_ptr->inp_vel3)
      + .6588 * (omni_state_ptr->inp_vel1 + omni_state_ptr->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state_ptr->out_vel1 + 2.5282 * omni_state_ptr->out_vel2
          - 0.7776 * omni_state_ptr->out_vel3);  //cutoff freq of 20 Hz
  //更新历史速度和位置缓存
  omni_state_ptr->pos_hist2 = omni_state_ptr->pos_hist1;
  omni_state_ptr->pos_hist1 = omni_state_ptr->position;
  omni_state_ptr->inp_vel3 = omni_state_ptr->inp_vel2;
  omni_state_ptr->inp_vel2 = omni_state_ptr->inp_vel1;
  omni_state_ptr->inp_vel1 = vel_buff;
  omni_state_ptr->out_vel3 = omni_state_ptr->out_vel2;
  omni_state_ptr->out_vel2 = omni_state_ptr->out_vel1;
  omni_state_ptr->out_vel1 = omni_state_ptr->velocity;
 
  //~ // Set forces if locked
   if (omni_state_ptr->lock == true) {
    omni_state_ptr->force = 0.04 * omni_state_ptr->units_ratio * (omni_state_ptr->lock_pos - omni_state_ptr->position)
        - 0.001 * omni_state_ptr->velocity;
  }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  //与位置变换保持一致，确保力的方向与位移方向匹配,存在问题：位置和力的坐标系变换不一致
  feedback[0] = omni_state_ptr->force[0];
  feedback[1] = omni_state_ptr->force[2];
  feedback[2] = -omni_state_ptr->force[1];
  //feedback[0] = -0.134654 feedback[1] = 1.03594 feedback[2] = 0.185584
  hdSetDoublev(HD_CURRENT_FORCE, feedback);
  //先设置再获取？
  hdGetDoublev(HD_CURRENT_FORCE, omni_state_ptr->forceGetOrigin);

  //结束当前帧
  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }
// 静态断言确保 joints 数组大小正确
// static_assert(sizeof(omni_state->joints) == 3 * sizeof(double), "joints array size mismatch");
  /*
   * t[0] 占位符
   * t[1]  joints[0] waist
   * t[2]  joints[1] shoulder
   * t[3]  joints[2] - joints[1] 表示差值角度，或许是特定机械结构造成？ elbow,相对于shoulder的角度差
   * t[4]  gimbal_angles[0] yaw
   * t[5]  gimbal_angles[1] pitch
   * t[6]  gimbal_angles[2] roll
  */
  double t[7] = { 0., omni_state_ptr->joints[0], omni_state_ptr->joints[1],
      omni_state_ptr->joints[2] - omni_state_ptr->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state_ptr->thetas[i] = t[i];
    // double y = omni_state->joints.m_p[0];
  return HD_CALLBACK_CONTINUE;
}
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  // 用于存储错误信息的结构体
  HDErrorInfo error;
  // 获取设备支持的校准方式
  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

  // 检查是否支持编码器复位校准方式
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }  // 检查是否支持墨水井校准方式 默认校准方式
  else if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }// 检查是否支持自动校准方式
  else if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }
  // 如果选择了编码器复位校准方式，则执行相应的校准过程
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      // 执行校准更新
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)");
      // 检查设备错误
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);

    ROS_INFO("Calibration complete.");
  }

  // 持续检查校准状态，直到校准成功
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);// 延迟1秒
    // 根据不同的校准状态，执行相应的操作
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }else{
      ROS_FATAL("Unknown calibration status");
    }
  }
}
void *ros_publish_omni(void *ptr) {
  // 将void*类型的ptr转换为PhantomROS*类型，以便于操作PhantomROS对象
  PhantomROS *phantom_ros = (PhantomROS *) ptr;
  
  // 从ROS参数服务器获取发布频率，如果没有设置，则使用默认值1000Hz
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 1000);
  ROS_INFO("Publishing PHANTOM state at [%d] Hz", publish_rate);
  // publish_rate = 2;
  ros::Rate loop_rate(publish_rate);

  // 创建并启动一个ROS异步旋转器，用于自动处理ROS消息传递
  ros::AsyncSpinner spinner(2);
  spinner.start();
  // 主循环，持续发布Phantom机器人的状态信息，直到ROS节点被关闭
  while (ros::ok()) {
    phantom_ros->publish_omni_state_and_pose_stampe();
    phantom_ros->publish_omni_button_event();
    phantom_ros->publish_omni_joint_state();
    phantom_ros->print_omni_state();
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "omni_haptic_node");
  // OmniState *state = NULL;
  // OmniState *state = new OmniState();
  // PhantomROS omni_ros;
  PhantomROS phantom_ros;
  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  // HDstring target_dev = HD_DEFAULT_DEVICE;
  // string dev_string;
  ros::NodeHandle nh("~");
  HHD hHD = hdInitDevice("Default Device");
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device");
    return -1;
  }
  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  //打开力输出
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to start the scheduler"); //, &error);
    return -1;
  }
  //Phantom校准
  HHD_Auto_Calibration();
  if(!phantom_ros.init()){
    ROS_ERROR("Failed to initialize phantom_ros");
    return -1;
  }
  // 将omni_state_callback回调函数异步地注册到调度期中执行，不会阻塞调用者等待其完成
  // hdScheduleAsynchronous(omni_state_callback, phantom_ros.OmniStatePtr,HD_MAX_SCHEDULER_PRIORITY);
  hdScheduleAsynchronous(omni_state_callback, &phantom_ros,HD_MAX_SCHEDULER_PRIORITY);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish_omni, (void*) &phantom_ros);
  pthread_join(publish_thread, NULL);
  // ros_publish(&phantom_ros);
  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
