#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Wrench.h>
#include "geometry_msgs/WrenchStamped.h"
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include <pthread.h>

float prev_time;
int calibrationStyle;
bool enable_debug_print = true;
int print_counter = 0;

struct OmniState {
  // hduVector3Dd表示一个三维向量的类
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  // 输入的速度，可能是从外部设备接收到的速度值
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;//hduVector3Dd 应该是一个包含 3 个 double 类型元素 的向量 double joints[3]
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
};
class PhantomROS {

public:
  ros::NodeHandle n;
  ros::Publisher state_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher button_publisher;
  ros::Publisher joint_publisher;
  ros::Subscriber haptic_sub;
  std::string omni_name, ref_frame, units;

  OmniState *state;
//以下中文注释为我自己注释，不保证正确性
// 初始化一个 Omni 设备的状态，并设置相关的 ROS 参数和话题
  void init(OmniState *s) {
    ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));

    ROS_INFO("================Initializing Phantom ROS node for [%s]\n", omni_name.c_str());
    ROS_INFO("===============Using reference frame [%s]\n", ref_frame.c_str());
    ROS_INFO("===============Using units [%s]\n", units.c_str());
    //Publish button state on NAME/button
    //创建一个按钮状态的话题，phantom/button，并发布 OmniButtonEvent 消息，队列长度为 100。
    std::ostringstream stream1;
    stream1 << omni_name << "/button";
    std::string button_topic = std::string(stream1.str());
    button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100);

    //Publish on phantom/state
    //创建一个状态话题，格式为 phantom/state，并发布 OmniState 消息，队列长度为 1。
    std::ostringstream stream2;
    stream2 << omni_name << "/state";
    std::string state_topic_name = std::string(stream2.str());
    state_publisher = n.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1);

    //Subscribe to NAME/force_feedback
    // 订阅 omni_name/force_feedback 话题，回调函数为 force_callback，队列长度为 1。
    std::ostringstream stream3;
    stream3 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

    //Publish on NAME/pose
    // 创建一个姿态话题，格式为 omni_name/pose，并发布 PoseStamped 消息，队列长度为 1。
    std::ostringstream stream4;
    stream4 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1);

    //Publish on NAME/joint_states
    // 创建一个关节状态话题，格式为 /joint_states_omni，并发布 JointState 消息，队列长度为 1。
    std::ostringstream stream5;
    stream5 << "/joint_states_omni";
    // stream5 << omni_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);

    // 初始化按钮状态：
    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    //buttons_prev[] 用于记录上一次的按钮状态,previous
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
    state->lock = false;
    state->close_gripper = false;
    state->lock_pos = zeros;
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHANTOM position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/
  // omni_msgs::OmniFeedbackConstPtr类型别名
  //定义为：boost::shared_ptr<omni_msgs::OmniFeedback const>
  //即指向 omni_msgs::OmniFeedback 常量对象的智能指针。？
  void force_callback(const omni_msgs::OmniFeedbackConstPtr& omnifeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    //力反馈处理：
    // 从 omnifeed 中获取力值，并减去一个与速度成正比的小量（0.001 * 速度），以增加稳定性。
    //更新 state->force 数组中的三个分量（x, y, z）。
    state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
    state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];
    //位置锁定:
    //直接将 omnifeed 中的位置值赋给 state->lock_pos 数组中的三个分量（x, y, z）。
    //位置同步：确保 state 对象中的位置信息与 omnifeed 提供的位置信息保持一致，实现位置的实时更新。
    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
  }
  /*新加begin*/
  void publish_omni_button() {
    // Build the button msg
    //初始化消息对象
    omni_msgs::OmniButtonEvent button_msg;
    // Button 0
    // 设置按钮0的状态
    // button_msg.buttons[0] = state->buttons[0];
    // Button 1
    // 设置按钮1的状态
    // button_msg.buttons[1] = state->buttons[1];
  }
  void subcribe_omni_feedback() {
    // Build the feedback msg
    //初始化消息对象
    omni_msgs::OmniFeedback feedback_msg;
    // Position
    // 获取位置信息：
    feedback_msg.position.x = state->position[0];
    feedback_msg.position.y = state->position[1];
    feedback_msg.position.z = state->position[2];
    // Orientation
    // 获取姿态信息
  }
  /*新加end*/
  void publish_omni_state() {
    //空指针检查
    if (!state){
      ROS_ERROR("State pointer is null!,skipping publish");
    }
    const ros::Time current_time = ros::Time::now();//统一时间戳

    // Build OmniState msg
    omni_msgs::OmniState state_msg;
    // 设置锁定状态和夹爪状态
    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;
    // Position
    state_msg.pose.position.x = state->position[0];
    state_msg.pose.position.y = state->position[1];
    state_msg.pose.position.z = state->position[2];

    // Orientation
    state_msg.pose.orientation.x = state->rot.v()[0];
    state_msg.pose.orientation.y = state->rot.v()[1];
    state_msg.pose.orientation.z = state->rot.v()[2];
    state_msg.pose.orientation.w = state->rot.s();

    // Velocity
    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];
    // TODO: Append Current to the state msg
    // 设置时间戳：
    // state_msg.header.stamp = ros::Time::now();
    state_msg.header.stamp = current_time;
    //发布OmniState msg
    state_publisher.publish(state_msg);


    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    // joint_state.header.stamp = ros::Time::now();
    joint_state.header.stamp = current_time;//统一时间戳
    joint_state.name.resize(6);
    joint_state.position.resize(6);

    //waist 腰部
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state->thetas[1];
    //shoulder 肩；肩部
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2];
    //elbow肘；肘部
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3];
    //yaw 偏航
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -state->thetas[4] + M_PI;
    //pitch 俯仰；（飞机等的）上下摆动
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
    //roll 翻滚；侧滚
    joint_state.name[5] = "roll";
    joint_state.position[5] = -state->thetas[6] - M_PI;

    //发布消息
    joint_publisher.publish(joint_state);

    // Build the pose msg
    //构建并发布 PoseStamped 消息
    geometry_msgs::PoseStamped pose_msg;
    // pose_msg.header = state_msg.header;
    pose_msg.header.stamp = current_time;//统一时间戳
    pose_msg.header.frame_id = ref_frame;
    //设置位置信息并转换单位
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;//没用到单位转换比率？

    //发布消息
    pose_publisher.publish(pose_msg);

    //处理按钮事件
    //如果2个按钮有按下的变化，则发布按钮事件
    // if ((state->buttons[0] != state->buttons_prev[0]) 
    //     or (state->buttons[1] != state->buttons_prev[1]))
    // {
    //   omni_msgs::OmniButtonEvent button_event;
    //   button_event.grey_button = state->buttons[0];
    //   button_event.white_button = state->buttons[1];
    //   if (state->buttons[0] == 1) {
    //     state->close_gripper = !(state->close_gripper);
    //   }
    //   if (state->buttons[1] == 1) {
    //     state->lock = !(state->lock);
    //   }
    //   state->buttons_prev[0] = state->buttons[0];
    //   state->buttons_prev[1] = state->buttons[1];

    //   button_publisher.publish(button_event);
    // }
    // Handle button events
    //只在按下时触发
    bool grey_button_rising = (state->buttons[0] == 1 && state->buttons_prev[0] == 0);
    bool white_button_rising = (state->buttons[1] == 1 && state->buttons_prev[1] == 0);
    if (grey_button_rising || white_button_rising)
    {
        omni_msgs::OmniButtonEvent button_event;
        button_event.grey_button = state->buttons[0];
        button_event.white_button = state->buttons[1];
        button_publisher.publish(button_event);

        if (grey_button_rising) {
            state->close_gripper = !state->close_gripper;
        }
        if (white_button_rising) {
            state->lock = !state->lock;
        }

        // Update previous button states
        state->buttons_prev[0] = state->buttons[0];
        state->buttons_prev[1] = state->buttons[1];
    }
  }

void print_omni_state() {
    if (!enable_debug_print) return;
    
    if (++print_counter % 10 != 0) return;  // 控制输出频率

    ROS_INFO(">--------%sRunning--------<", omni_name.c_str());
    ROS_INFO("ref_frame = %s\tunits = %s\n", ref_frame.c_str(), units.c_str());

    for (int i = 0; i < 3; ++i) {
        ROS_INFO("position[%d] = %f\t", i, state->position[i]);
    }

    for (int i = 0; i < 7; ++i) {
        ROS_INFO("thetas[%d] = %f\t", i, state->thetas[i]);
    }

    for (int i = 0; i < 2; ++i) {
        ROS_INFO("buttons[%d] = %d\t", i, state->buttons[i]);
    }

    for (int i = 0; i < 3; ++i) {
        ROS_INFO("velocity[%d] = %f\t", i, state->velocity[i]);
    }

    for (int i = 0; i < 3; ++i) {
        ROS_INFO("force[%d] = %f\t", i, state->force[i]);
    }
    std::cout << "Rotation quaternion: " << state->rot << "\n";
}
};


//mni_state_callback 是一个回调函数，用于处理 Omni 设备的状态更新。
//它在每次设备状态变化时被调用，负责获取设备的各种状态信息并进行相应的处理。
HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  OmniState *omni_state = static_cast<OmniState *>(pUserData);

  //检查设备是否需要重新校准，如果需要则使用指定的calibrationstyle方式校准
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
    //调用 hdBeginFrame 开始处理当前帧
  hdBeginFrame(hdGetCurrentDevice());

  // Get transform and angles
  //   获取当前设备末端的位置和姿态（4x4变换矩阵）；
  // 获取当前各关节的角度值，保存到 omni_state->joints
  hduMatrix transform;
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);//获取一组关节角度值？
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  // Notice that we are inverting the Z-position value and changing Y <---> Z
  //1. 反转 Z 轴位置值 相当于绕 X 轴旋转 180°（或绕 Y 轴旋转 180°），使 Z 轴方向与原方向相反。
  // 2. 交换 Y 轴和 Z 轴 相当于绕 X 轴旋转 90°，使原 Y 轴方向变为新 Z 轴方向，原 Z 轴方向变为新 Y 轴方向。
  // 总的：绕 X 轴旋转 90° + 关于 XY 平面的镜像（或类似操作），具体取决于坐标系的初始手性
  // Position
  omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  omni_state->position /= omni_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  //调试rotation_offset 和 rotation
  // Velocity estimation
  // 利用当前位置和前两次历史位置计算瞬时速度
  hduVector3Dd vel_buff(0, 0, 0);
  // 使用二阶后向差分法估算速度；
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  //应用低通滤波器平滑速度输出（截止频率20Hz）；
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  //更新历史速度和位置缓存
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  //~ // Set forces if locked
  //~ if (omni_state->lock == true) {
    //~ omni_state->force = 0.04 * omni_state->units_ratio * (omni_state->lock_pos - omni_state->position)
        //~ - 0.001 * omni_state->velocity;
  //~ }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  //与位置变换保持一致，确保力的方向与位移方向匹配,存在问题：位置和力的坐标系变换不一致
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  //Get buttons
  int nButtons = 0;//应该是位掩码的形式
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;//0b0001 & 0b0011 = 1
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;//0b0010 & 0b0011 = 1

  //调用 hdBeginFrame 结束当前帧
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
  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
    // double y = omni_state->joints.m_p[0];
  return HD_CALLBACK_CONTINUE;
}

/**
 * @brief 执行HHD设备的自动校准程序
 * 
 * 该函数首先确定支持的校准方式，然后根据支持的校准方式选择合适的校准策略。
 * 它会尝试进行自动校准，并在必要时提示用户进行手动干预以完成校准过程。
 */
void HHD_Auto_Calibration() {
  // 支持的校准方式的整数表示
  int supportedCalibrationStyles;
  // 用于存储错误信息的结构体
  HDErrorInfo error;

  // 获取设备支持的校准方式
  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

  // 检查是否支持编码器复位校准方式
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }

  // 检查是否支持墨水井校准方式
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }

  // 检查是否支持自动校准方式
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
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
    // 延迟1秒
    usleep(1e6);

    // 根据不同的校准状态，执行相应的操作
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      ROS_FATAL("Unknown calibration status");
  }
}
/**
 * @brief 发布Phantom机器人的状态信息到ROS话题
 * 
 * 该函数是一个无限循环，负责以指定的频率发布Phantom机器人的状态信息到ROS网络中。
 * 它首先从ROS参数服务器获取发布频率，然后以该频率执行发布操作。
 * 
 * @param ptr void* 类型的指针，预期指向一个PhantomROS实例。这样设计是为了让线程函数能够接受任何类型的参数。
 * @return void* 总是返回NULL，因为这个函数不返回任何有意义的值。
 */
void *ros_publish(void *ptr) {
  // 将void*类型的ptr转换为PhantomROS*类型，以便于操作PhantomROS对象
  PhantomROS *omni_ros = (PhantomROS *) ptr;

  // 从ROS参数服务器获取发布频率，如果没有设置，则使用默认值1000Hz
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 1000);

  // 打印发布频率信息到ROS日志系统，便于调试和信息记录
  ROS_INFO("Publishing PHaNTOM state at [%d] Hz", publish_rate);

  // 创建一个ROS率对象，用于控制循环的执行频率
  // publish_rate = 1;
  ros::Rate loop_rate(publish_rate);

  // 创建并启动一个ROS异步旋转器，用于自动处理ROS消息传递
  ros::AsyncSpinner spinner(2);
  spinner.start();
  int i = 0;
  // 主循环，持续发布Phantom机器人的状态信息，直到ROS节点被关闭
  while (ros::ok()) {

    omni_ros->publish_omni_state();
    omni_ros->print_omni_state();
    loop_rate.sleep();

  }
  // 循环结束后返回NULL，表示线程执行完毕
  return NULL;
}

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "omni_haptic_node");
  OmniState state;
  PhantomROS omni_ros;
  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  HHD hHD;
  // HDstring target_dev = HD_DEFAULT_DEVICE;
  // string dev_string;
  ros::NodeHandle nh("~");
  std::string device_name;
  int device_ret = nh.getParam("device_name", device_name);
  HDstring target_dev = device_name.c_str();
  // if (device_ret)
   {
    // ROS_ERROR("device name found");
    ROS_INFO("device name: %s", device_name.c_str());
    std::cout << "device name: " << device_name << std::endl;
  }
  hHD = hdInitDevice(target_dev);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
    return -1;
  }
  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to start the scheduler"); //, &error);
    return -1;
  }
  
  HHD_Auto_Calibration();

  omni_ros.init(&state);
  //&state会传递给omni_state_callback
  hdScheduleAsynchronous(omni_state_callback, &state,HD_MAX_SCHEDULER_PRIORITY);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
