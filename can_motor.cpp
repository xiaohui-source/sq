#include "motor_control/can_motor.h"
#include <unistd.h>

using namespace std;

can_motors::can_motors(ros::Publisher& pub, const can_activate_ptr* motor_set_list, int can_index_number_)
  : can_motors_pub(pub), can_active_list(motor_set_list), can_index_number(can_index_number_)
{
  // 1、配置电机id等数据

  can_index_list = new int[can_index_number];
  for (int i = 0; i < can_index_number; i++)
  {
    can_id_number[i] = sizeof(can_active_list[i].motor_set) / sizeof(can_active_list[i].motor_set[0]);
    motor_number = motor_number + can_id_number[i];
    can_index_list[i] = can_active_list[i].can_index;
  }

  // 2、can盒硬件初始化
  CanInit();  //初始化设备

  // 3、配置电机并启动
  //拉升
  motor_length = new CanMotor_LengthClass(motor_set_list[1].motor_set[0]);
  motor_length->MotorStartEncode();

  Can_Transmit(&motor_length->send_data, 1);

  //平移
  motor_trans = new CanMotor_ServoClass(motor_set_list[0].motor_set[1], 0.63);
  for (int i = 0; i < 10; i++)
  {
    motor_trans->MotorModeEncode(MOTOR_Servo_BUSTYPE);  //选择总线类型,can
    Can_Transmit(&motor_trans->send_data, 0);
    motor_trans->MotorModeEncode(MOTOR_Servo_ENABLE);  //使能电机
    Can_Transmit(&motor_trans->send_data, 0);
    motor_trans->MotorModeEncode(MOTOR_MODE_VELOCITY);  //设置模式为速度模式
    Can_Transmit(&motor_trans->send_data, 0);
  }
  //回转
  motor_yaw = new CanMotor_ServoClass(motor_set_list[0].motor_set[0], 3.2);
  for (int i = 0; i < 10; i++)
  {
    motor_yaw->MotorModeEncode(MOTOR_Servo_BUSTYPE);
    Can_Transmit(&motor_yaw->send_data, 0);
    motor_yaw->MotorModeEncode(MOTOR_Servo_ENABLE);
    Can_Transmit(&motor_yaw->send_data, 0);
    motor_yaw->MotorModeEncode(MOTOR_MODE_VELOCITY);
    Can_Transmit(&motor_yaw->send_data, 0);
  }
  //俯仰
  motor_pitch = new CanMotor_ServoClass(motor_set_list[0].motor_set[2], 2.4);
  for (int i = 0; i < 10; i++)
  {
    motor_pitch->MotorModeEncode(MOTOR_Servo_BUSTYPE);
    Can_Transmit(&motor_pitch->send_data, 0);
    motor_pitch->MotorModeEncode(MOTOR_Servo_ENABLE);
    Can_Transmit(&motor_pitch->send_data, 0);
    motor_pitch->MotorModeEncode(MOTOR_MODE_VELOCITY);
    Can_Transmit(&motor_pitch->send_data, 0);
  }

  motor_length->MotorSetZeroEncode();
  Can_Transmit(&motor_length->send_data, 1);

  // 4、开启发送和接受线程
  cout << "线程开始" << endl;
  std::thread thread_send(&can_motors::thread_send_callback, this);
  std::thread thread_receive(&can_motors::thread_receive_callback, this);
  std::thread thread_timer(&can_motors::thread_timer_callback, this);

  thread_send.detach();
  thread_receive.detach();
  thread_timer.detach();
}

can_motors::~can_motors()
{
}

// can盒初始化
void can_motors::CanInit()
{
  // 1、打开can盒
  int state = 0;
  state = VCI_OpenDevice(VCI_USBCAN2A, 0, 0);
  if (state == 1)  //打开设备
  {
    printf(">>open deivce success!\n");  //打开设备成功
  }
  else if (state == -1)
  {
    printf(">>no can device!\n");
  }
  else
  {
    printf(">>open deivce fail!\n");  //这个就算成功打开，还是为0
  }
  // 2、初始化两路can口
  std::cout << "初始化can口" << can_index_number << std::endl;

  for (int i = 0; i < can_index_number; i++)
  {
    // 2.1 can1初始化
    VCI_INIT_CONFIG config_can;
    config_can.AccCode = 0;
    config_can.AccMask = 0xFFFFFFFF;
    config_can.Filter = 1;  //接收所有帧
    config_can.Mode = 0;    //正常模式
    if (can_active_list[i].baud_rate == 1000)
    {
      config_can.Timing0 = 0x00; /*波特率1000 Kbps*/
      config_can.Timing1 = 0x14;
      std::cout << "1000" << std::endl;
    }
    else if (can_active_list[i].baud_rate == 500)
    {
      config_can.Timing0 = 0x00; /*波特率500 Kbps*/
      config_can.Timing1 = 0x1c;
      std::cout << "500" << std::endl;
    }

    if (VCI_InitCAN(VCI_USBCAN2, 0, can_active_list[i].can_index, &config_can) != 1)
    {
      // printf(">>Init CAN1 error\n");
      cout << ">>Init CAN" << can_active_list[i].can_index << "error" << endl;
      VCI_CloseDevice(VCI_USBCAN2, can_active_list[i].can_index);
    }
    else
    {
      cout << ">>Init CAN" << can_active_list[i].can_index << "success" << endl;
      // printf(">>Init CAN1 success\n");
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, can_active_list[i].can_index) != 1)
    {
      cout << ">>Start CAN" << can_active_list[i].can_index << "error" << endl;
      // printf(">>Start CAN1 error\n");
      VCI_CloseDevice(VCI_USBCAN2, can_active_list[i].can_index);
    }
    else
    {
      cout << ">>Start CAN" << can_active_list[i].can_index << "success" << endl;
      // printf(">>Start CAN1 success\n");
    }
  }

  // cout << "初始化完成"<< endl;
}

void can_motors::Can_Transmit(VCI_CAN_OBJ* send_data, int can_index)
{
  CanBox_ThreadMutex.lock();

  if (VCI_Transmit(VCI_USBCAN2, 0, can_index, send_data, 1) == 1)
  {
    // printf("send data success:3\n");
  }
  else
  {
    printf("send data fail\n");
  }
  CanBox_ThreadMutex.unlock();
  usleep(1);
}

int can_motors::Can_Recive(VCI_CAN_OBJ* rec_data, int can_index)
{
  CanBox_ThreadMutex.lock();
  int reclen = VCI_Receive(VCI_USBCAN2, 0, can_index, rec_data, 50, 200);
  CanBox_ThreadMutex.unlock();
  usleep(1);  //让线程停止1毫秒，防止较快的线程总是占用硬件
  return reclen;
}

// 电机状态信息（位置、速度、力矩）发布
void can_motors::thread_timer_callback()
{
  // 定义定时器周期
  std::chrono::milliseconds interval(10);

  motor_control::motors_fdb motors_fdb_;
  motors_fdb_.length_motor.id = motor_length->motor_set.motor_id;
  motors_fdb_.trans_motor.id = motor_trans->motor_set.motor_id;
  motors_fdb_.yaw_motor.id = motor_yaw->motor_set.motor_id;
  motors_fdb_.pitch_motor.id = motor_pitch->motor_set.motor_id;
  // 循环执行定时器
  while (true)
  {
    motors_fdb_.length_motor.position = motor_length->pos_fdb;
    motors_fdb_.length_motor.speed = motor_length->spd_fdb;
    motors_fdb_.length_motor.torque = motor_length->tor_fdb;

    motors_fdb_.trans_motor.position = motor_trans->pos_fdb;
    motors_fdb_.trans_motor.speed = motor_trans->spd_fdb;
    motors_fdb_.trans_motor.torque = motor_trans->tor_fdb;

    motors_fdb_.yaw_motor.position = motor_yaw->pos_fdb;
    motors_fdb_.yaw_motor.speed = motor_yaw->spd_fdb;
    motors_fdb_.yaw_motor.torque = motor_yaw->tor_fdb;

    motors_fdb_.pitch_motor.position = motor_pitch->pos_fdb;
    motors_fdb_.pitch_motor.speed = motor_pitch->spd_fdb;
    motors_fdb_.pitch_motor.torque = motor_pitch->tor_fdb;

    can_motors_pub.publish(motors_fdb_);
    // 等待定时器周期
    std::this_thread::sleep_for(interval);
  }
}

// 电机状态信息获取
void can_motors::thread_receive_callback()
{
  while (1)
  {
    // std::cout << "===============receive thread" << std::endl;
    // ROS_INFO("++++++++++rec_callback++++++++++");
    for (int i = 0; i < can_index_number; i++)
    {
      VCI_CAN_OBJ* rec_data_buff = new VCI_CAN_OBJ[50];
      // std::cout << "can_index_number:"<< can_index_number << "can index:" <<
      // can_index_list[i] <<"i:"<<i<< std::endl;
      int reclen = Can_Recive(rec_data_buff, can_index_list[i]);  //这里单独处理吧
      if (reclen > 0)
      {  //调用接收函数，如果有数据，则进行处理
        for (int j = 0; j < reclen; j++)
        {
          if (rec_data_buff[j].ID == 0)
          {  //提升电机
            motor_length->MotorRecDecode(rec_data_buff[j]);
          }
          else if (rec_data_buff[j].ID == 1539)
          {  //和利时电机
            motor_trans->MotorRecDecode(rec_data_buff[j]);
          }
          else if (rec_data_buff[j].ID == 1537)
          {
            motor_yaw->MotorRecDecode(rec_data_buff[j]);
          }
          else if (rec_data_buff[j].ID == 1538)
          {
            motor_pitch->MotorRecDecode(rec_data_buff[j]);
          }
        }
      }
      delete[] rec_data_buff;
    }
  }
}

void can_motors::thread_send_callback()
{
  while (1)
  {
    // std::cout << "==================send thread" << std::endl;
    // ROS_INFO("++++++++++send_callback++++++++++");

    for (int i = 0; i < can_index_number; i++)
    {
      for (int j = 0; j < can_id_number[i]; j++)
      {
        if (can_active_list[i].motor_set[j].motor_id == 4)
        {
          // 1 拉绳电机发送
          // 1.1 限范围
          if (motor_length->pos_fdb > motor_length->motor_set.limit_max)
          {  //位置超过最大值
            if (motor_length->motor_mode == 1)
            {  //位置模式
              if (motor_length->pos_des > motor_length->motor_set.limit_max)
              {  //还想往外走
                motor_length->pos_des = motor_length->motor_set.limit_max;
              }
            }
            else if (motor_length->motor_mode == 2)
            {  //速度模式
              if (motor_length->spd_des > 0)
              {
                motor_length->spd_des = 0;
              }
            }
            else if (motor_length->motor_mode == 3 || motor_length->motor_mode == 4 || motor_length->motor_mode == 5 ||
                     motor_length->motor_mode == 6 || motor_length->motor_mode == 7 || motor_length->motor_mode == 8)
            {  //力矩模式
              if (motor_length->tor_des > 0)
              {
                motor_length->tor_des = 0;
              }
            }
          }
          else if (motor_length->pos_fdb < motor_length->motor_set.limit_min)
          {  //位置低于最小值
            if (motor_length->motor_mode == 1)
            {  //位置模式
              if (motor_length->pos_des < motor_length->motor_set.limit_min)
              {
                motor_length->pos_des = motor_length->motor_set.limit_min;
              }
            }
            else if (motor_length->motor_mode == 2)
            {  //速度模式
              if (motor_length->spd_des < 0)
              {
                motor_length->spd_des = 0;
              }
            }
            else if (motor_length->motor_mode == 3 || motor_length->motor_mode == 4 || motor_length->motor_mode == 5 ||
                     motor_length->motor_mode == 6 || motor_length->motor_mode == 7 || motor_length->motor_mode == 8)
            {  //力矩模式

              if (motor_length->tor_des < 0)
              {
                motor_length->tor_des = 0;
              }
            }
          }
          motor_length->MotorControlEncode();
          // cout << "11111111111111:"<<motor_length->pos_des<<endl;
          // cout << "11111111111112:"<<motor_length->spd_des<<endl;
          // cout << "11111111111113:"<<motor_length->tor_des<<endl;
          Can_Transmit(&motor_length->send_data, can_active_list[i].can_index);
        }
        if (can_active_list[i].motor_set[j].motor_id == 3)
        {
          // 2.平移电机发送
          // 2.1 模式切换
          if (motor_trans->motor_mode != motor_trans->motor_mode_buff)
          {
            for (int k = 0; k < 10; k++)
            {
              motor_trans->MotorModeEncode(motor_trans->motor_mode);
              Can_Transmit(&motor_trans->send_data, can_active_list[i].can_index);
            }
            motor_trans->motor_mode_buff = motor_trans->motor_mode;
          }
          // 2.2 限位
          if (motor_trans->pos_fdb > motor_trans->motor_set.limit_max)
          {  //位置超过最大值
            if (motor_trans->motor_mode == 1)
            {  //位置模式
              if (motor_trans->pos_des > motor_trans->motor_set.limit_max)
              {  //还想往外走
                motor_trans->pos_des = motor_trans->motor_set.limit_max;
              }
            }
            else if (motor_trans->motor_mode == 2)
            {  //速度模式
              if (motor_trans->spd_des > 0)
              {
                motor_trans->spd_des = 0;
              }
            }
            else
            {  //力矩模式
              if (motor_trans->tor_des > 0)
              {
                motor_trans->tor_des = 0;  //力矩模式的限制,要想一下.尽量还是位置的控制
              }
            }
          }
          else if (motor_trans->pos_fdb < motor_trans->motor_set.limit_min)
          {  //位置低于最小值
            if (motor_trans->motor_mode == 1)
            {  //位置模式
              if (motor_trans->pos_des < motor_trans->motor_set.limit_min)
              {
                motor_trans->pos_des = motor_trans->motor_set.limit_min;
              }
            }
            else if (motor_trans->motor_mode == 2)
            {  //速度模式
              if (motor_trans->spd_des < 0)
              {
                motor_trans->spd_des = 0;
              }
            }
            else
            {  //力矩模式
              if (motor_trans->tor_des < 0)
              {
                motor_trans->tor_des = 0;
              }
            }
          }
          // 2.3 发送
          motor_trans->MotorControlEncode();
          Can_Transmit(&motor_trans->send_data, can_active_list[i].can_index);
        }
        if (can_active_list[i].motor_set[j].motor_id == 1)
        {
          // 3.回转电机发送
          // 3.1 模式切换
          if (motor_yaw->motor_mode != motor_yaw->motor_mode_buff)
          {
            for (int k = 0; k < 10; k++)
            {
              motor_yaw->MotorModeEncode(motor_yaw->motor_mode);
              Can_Transmit(&motor_yaw->send_data, can_active_list[i].can_index);
            }
            motor_yaw->motor_mode_buff = motor_yaw->motor_mode;
          }
          // 3.2 限位
          if (motor_yaw->pos_fdb > motor_yaw->motor_set.limit_max)
          {  //位置超过最大值
            if (motor_yaw->motor_mode == 1)
            {  //位置模式
              if (motor_yaw->pos_des > motor_yaw->motor_set.limit_max)
              {  //还想往外走
                motor_yaw->pos_des = motor_yaw->motor_set.limit_max;
              }
            }
            else if (motor_yaw->motor_mode == 2)
            {  //速度模式
              if (motor_yaw->spd_des > 0)
              {
                motor_yaw->spd_des = 0;
              }
            }
            else
            {  //力矩模式
              if (motor_yaw->tor_des > 0)
              {
                motor_yaw->tor_des = 0;  //力矩模式的限制,要想一下.尽量还是位置的控制
              }
            }
          }
          else if (motor_yaw->pos_fdb < motor_yaw->motor_set.limit_min)
          {  //位置低于最小值
            if (motor_yaw->motor_mode == 1)
            {  //位置模式
              if (motor_yaw->pos_des < motor_yaw->motor_set.limit_min)
              {
                motor_yaw->pos_des = motor_yaw->motor_set.limit_min;
              }
            }
            else if (motor_yaw->motor_mode == 2)
            {  //速度模式
              if (motor_yaw->spd_des < 0)
              {
                motor_yaw->spd_des = 0;
              }
            }
            else if (motor_yaw->motor_mode == 3 || motor_yaw->motor_mode == 4 || motor_yaw->motor_mode == 5 ||
                     motor_yaw->motor_mode == 6 || motor_yaw->motor_mode == 7 || motor_yaw->motor_mode == 8)
            {  //力矩模式
              if (motor_yaw->tor_des < 0)
              {
                motor_yaw->tor_des = 0;
              }
            }
          }
          // 3.3 发送
          // std::cout << "回转发送"<<endl;
          // ROS_INFO("+++++++++++++++++++++++++=yaw send");
          // std::cout << "回转发送："<< motor_yaw->pos_des << "id:"
          // <<can_active_list[i].can_index<<endl;
          motor_yaw->MotorControlEncode();
          // 						    cout << "send id:"
          // << motor_yaw->send_data.ID
          // << " "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[0]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[1]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[2]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[3]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[4]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[5]) << " "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[6]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_yaw->send_data.Data[7]) << " "
          // << endl;
          Can_Transmit(&motor_yaw->send_data, can_active_list[i].can_index);
          // std::cout << "回转完成"<<endl;
        }
        if (can_active_list[i].motor_set[j].motor_id == 2)
        {
          // 1、结合限制位置和命令，设置真正的期望命令模式
          // 4.俯仰电机发送
          // std::cout << "进入俯仰"<<endl;
          if (motor_pitch->motor_mode == 1)
          {
            motor_pitch->motor_mode = 2;
            if (motor_pitch->pos_fdb < motor_pitch->pos_des - 1)
            {  //就是这里可能导致的抖动，比如想去25,卡在了24.46,稍稍动一下就要启动速度1。所以把误差改打一点，只是测试用的
              motor_pitch->spd_des = 1;
            }
            else if (motor_pitch->pos_fdb > motor_pitch->pos_des + 1)
            {
              motor_pitch->spd_des = -1;
            }
            else
            {
              motor_pitch->spd_des = 0;
            }
          }

          // std::cout << "进入俯仰jieshu1"<<endl;

          // 4.1 限位
          if (motor_pitch->pos_fdb > motor_pitch->motor_set.limit_max)
          {  //位置超过最大值
            if (motor_pitch->motor_mode == 1)
            {  //位置模式
              if (motor_pitch->pos_des > motor_pitch->motor_set.limit_max)
              {  //还想往外走
                motor_pitch->pos_des = motor_pitch->motor_set.limit_max;
              }
            }
            else if (motor_pitch->motor_mode == 2)
            {  //速度模式
              if (motor_pitch->spd_des > 0)
              {
                motor_pitch->spd_des = 0;
              }
            }
            else if (motor_pitch->motor_mode == 3 || motor_pitch->motor_mode == 4 || motor_pitch->motor_mode == 5 ||
                     motor_pitch->motor_mode == 6 || motor_pitch->motor_mode == 7 || motor_pitch->motor_mode == 8)
            {  //力矩模式
              if (motor_pitch->tor_des > 0)
              {
                motor_pitch->spd_des = 0;
                motor_pitch->motor_mode = 2;
              }
            }
          }
          else if (motor_pitch->pos_fdb < motor_pitch->motor_set.limit_min)
          {  //位置低于最小值
            if (motor_pitch->motor_mode == 1)
            {  //位置模式
              if (motor_pitch->pos_des < motor_pitch->motor_set.limit_min)
              {
                motor_pitch->pos_des = motor_pitch->motor_set.limit_min;
              }
            }
            else if (motor_pitch->motor_mode == 2)
            {  //速度模式
              if (motor_pitch->spd_des < 0)
              {
                motor_pitch->spd_des = 0;
              }
            }
            else if (motor_pitch->motor_mode == 3 || motor_pitch->motor_mode == 4 || motor_pitch->motor_mode == 5 ||
                     motor_pitch->motor_mode == 6 || motor_pitch->motor_mode == 7 || motor_pitch->motor_mode == 8)
            {  //力矩模式
              if (motor_pitch->tor_des < 0)
              {
                motor_pitch->spd_des = 0;
                motor_pitch->motor_mode = 2;
              }
            }
          }

          // 4.2 模式切换发送
          // std::cout << "mode1:"<<motor_pitch->motor_mode<<endl;
          if (motor_pitch->motor_mode != motor_pitch->motor_mode_buff)
          {
            std::cout << "切换,mode1:" << motor_pitch->motor_mode << ",mode2:" << motor_pitch->motor_mode_buff << endl;

            for (int k = 0; k < 10; k++)
            {
              int pitch_motor_mode = motor_pitch->motor_mode;
              if (pitch_motor_mode == 4 || pitch_motor_mode == 5 || pitch_motor_mode == 6 || pitch_motor_mode == 7 ||
                  pitch_motor_mode == 8)
              {
                pitch_motor_mode = 3;
              }

              motor_pitch->MotorModeEncode(pitch_motor_mode);
              Can_Transmit(&motor_pitch->send_data, can_active_list[i].can_index);
            }
            motor_pitch->motor_mode_buff = motor_pitch->motor_mode;
          }

          // 4.3 发送
          // std::cout << "仰头发送"<<endl;
          // ROS_INFO("+++++++++++++++++++++++++=pitch send");
          motor_pitch->MotorControlEncode();
          // 						    cout << "send id:"
          // << motor_pitch->send_data.ID << " "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[0]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[1]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[2]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[3]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[4]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[5]) << " "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[6]) <<" "
          // << setfill('0') << setw(2) << hex <<
          // static_cast<int>(motor_pitch->send_data.Data[7]) << " "
          // << endl;
          Can_Transmit(&motor_pitch->send_data, can_active_list[i].can_index);
          // std::cout << "仰头:"<< motor_pitch->tor_des<<endl;
          // std::cout << "仰头完成"<<endl;
        }
      }
    }
  }
}

/*********************电机基类*************************/

CanMotor_BaseClass::CanMotor_BaseClass(const motor_set_ptr& motor_set_) : motor_set(motor_set_)
{
}

CanMotor_BaseClass::~CanMotor_BaseClass()
{
}

CanMotor_LengthClass::CanMotor_LengthClass(const motor_set_ptr& motor_set_) : CanMotor_BaseClass(motor_set_)
{
  send_data.ID = motor_set_.can_id;
  send_data.SendType = 1;
  send_data.RemoteFlag = 0;
  send_data.ExternFlag = 0;
  send_data.DataLen = 8;
}
void CanMotor_LengthClass::MotorStartEncode()
{
  // ROS_INFO("=============encode");
  send_data.Data[0] = 0xFF;
  send_data.Data[1] = 0xFF;
  send_data.Data[2] = 0xFF;
  send_data.Data[3] = 0xFF;
  send_data.Data[4] = 0xFF;
  send_data.Data[5] = 0xFF;
  send_data.Data[6] = 0xFF;
  send_data.Data[7] = 0xFC;
}

void CanMotor_LengthClass::MotorSetZeroEncode()
{
  send_data.Data[0] = 0xFF;
  send_data.Data[1] = 0xFF;
  send_data.Data[2] = 0xFF;
  send_data.Data[3] = 0xFF;
  send_data.Data[4] = 0xFF;
  send_data.Data[5] = 0xFF;
  send_data.Data[6] = 0xFF;
  send_data.Data[7] = 0xFE;
}

void CanMotor_LengthClass::MotorControlEncode()
{
  uint16_t s_p_int = float_to_uint(pos_des * motor_set.rate, -1433, 1433, 65535);
  uint16_t s_v_int = float_to_uint(spd_des * motor_set.rate, -1719, 1719, 4096);
  uint16_t s_t_int = float_to_uint(tor_des / motor_set.rate, -10, 10, 4096);

  uint16_t s_Kp_int = 0;
  uint16_t s_Kd_int = 0;

  if (motor_mode == 1)
  {  //位置模式
    s_Kp_int = float_to_uint(1, 0, 500, 4096);
    s_Kd_int = float_to_uint(1, 0, 5, 4096);
  }
  else if (motor_mode == 2)
  {  //速度模式
    s_Kp_int = float_to_uint(0, 0, 500, 4096);
    s_Kd_int = float_to_uint(1, 0, 5, 4096);
  }
  else if (motor_mode == 3 || motor_mode == 4 || motor_mode == 5 || motor_mode == 6 || motor_mode == 7 ||
           motor_mode == 8)
  {  //力矩模式
    s_Kp_int = float_to_uint(0, 0, 500, 4096);
    s_Kd_int = float_to_uint(0, 0, 5, 4096);
  }

  send_data.Data[0] = s_p_int >> 8;
  send_data.Data[1] = s_p_int & 0xFF;
  send_data.Data[2] = s_v_int >> 4;
  send_data.Data[3] = ((s_v_int & 0xF) << 4) + (s_Kp_int >> 8);
  send_data.Data[4] = s_Kp_int & 0xFF;
  send_data.Data[5] = s_Kd_int >> 4;
  send_data.Data[6] = ((s_Kd_int & 0xF) << 4) + (s_t_int >> 8);
  send_data.Data[7] = s_t_int & 0xFF;

  // send_data.Data[0] = 0x90;
  // send_data.Data[1] = 0x13;
  // send_data.Data[2] = 0x7F;
  // send_data.Data[3] = 0xF0;
  // send_data.Data[4] = 0x08;
  // send_data.Data[5] = 0x33;
  // send_data.Data[6] = 0x37;
  // send_data.Data[7] = 0xFF;

  // cout << "send_id:" << send_data.ID << " "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[0])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[1])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[2])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[3])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[4])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[5])
  // << " "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[6])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[7])
  // << " "
  // 	<< endl;
}
void CanMotor_LengthClass::MotorRecDecode(VCI_CAN_OBJ rec_data_buff)
{
  // can_motor_fdb[i].id = rec_data_buff.Data[0];
  uint16_t utemp = (static_cast<uint16_t>(rec_data_buff.Data[1]) << 8) | rec_data_buff.Data[2];
  pos_fdb = uint_to_float(utemp, 32768, 1433) / motor_set.rate;
  utemp = (static_cast<uint16_t>(rec_data_buff.Data[3]) << 4) | (rec_data_buff.Data[4] >> 4);
  spd_fdb = uint_to_float(utemp, 2048, 1719) / motor_set.rate;
  utemp = static_cast<uint32_t>(((rec_data_buff.Data[4] & 0x0F) << 8) | rec_data_buff.Data[5]);
  tor_fdb = uint_to_float(utemp, 2048, 10) * motor_set.rate;  //*0.05*10;

  // std::cout <<"id:"<< rec_data_buff.ID<<std::endl;
  // std::cout <<"id:"<< (rec_data_buff.Data[0] >> 4)<<std::endl;
  // std::cout <<"pos:"<< pos_fdb<<std::endl;
  // std::cout <<"spd:"<< spd_fdb<<std::endl;
  // std::cout <<"tor:"<< tor_fdb<<std::endl;

  // cout << "rec_id:" << rec_data_buff.ID << " "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[0]) <<" "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[1]) <<" "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[2]) <<" "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[3]) <<" "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[4]) <<" "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[5]) << " "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[6]) <<" "
  // << setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[7]) << " "
  // << setfill('0') << setw(4) << hex <<
  // static_cast<int>(static_cast<uint16_t>(rec_data_buff.Data[3]) << 4) << " "
  // << setfill('0') << setw(4) << hex << (rec_data_buff.Data[4]>>4) << " "
  // << endl;
}

CanMotor_ServoClass::CanMotor_ServoClass(const motor_set_ptr& motor_set_, float force_max_)
  : CanMotor_BaseClass(motor_set_), force_max(force_max_)
{
  send_data.ID = 256 + motor_set_.can_id;  // 103
  send_data.SendType = 1;
  send_data.RemoteFlag = 0;
  send_data.ExternFlag = 0;
  send_data.DataLen = 8;
}

void CanMotor_ServoClass::MotorModeEncode(int mode)
{
  if (mode == 4)
  {  //总线类型
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x24;
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x00;
    send_data.Data[4] = 0x00;
    send_data.Data[5] = 0x03;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }
  else if (mode == 5)
  {  //使能
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x24;
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x10;
    send_data.Data[4] = 0x00;
    send_data.Data[5] = 0x01;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }
  else if (mode == 6)
  {  //失能
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x24;
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x10;
    send_data.Data[4] = 0x00;
    send_data.Data[5] = 0x00;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }
  else if (mode == 1)
  {  //位置模式
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x24;
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x03;
    send_data.Data[4] = 0x00;
    send_data.Data[5] = 0x06;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }
  else if (mode == 2)
  {  //速度模式
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x24;
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x03;
    send_data.Data[4] = 0x00;
    send_data.Data[5] = 0x02;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }
  else if (mode == 3)
  {  //力矩模式
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x24;
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x03;
    send_data.Data[4] = 0x00;
    send_data.Data[5] = 0x01;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }

  // cout << "id:" << send_data.ID << " "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[0])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[1])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[2])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[3])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[4])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[5])
  // << " "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[6])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[7])
  // << " "
  // 	<< endl;
}

void CanMotor_ServoClass::MotorStartEncode()
{
  // send_data.Data[0] = 0xFF;
  // send_data.Data[1] = 0xFF;
  // send_data.Data[2] = 0xFF;
  // send_data.Data[3] = 0xFF;
  // send_data.Data[4] = 0xFF;
  // send_data.Data[5] = 0xFF;
  // send_data.Data[6] = 0xFF;
  // send_data.Data[7] = 0xFC;
}
void CanMotor_ServoClass::MotorControlEncode()
{
  if (motor_mode == 1)
  {  //位置模式
     // cout<< "send pos"<<endl;
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x3c;  //功能码
    send_data.Data[2] = 0x00;  // 100的速度
    send_data.Data[3] = 0x99;
    uint32_t data_uint = static_cast<uint32_t>(pos_des / 360 * 10000 * motor_set.rate);
    send_data.Data[4] = (data_uint & 0xff000000) >> 24;  //数据
    send_data.Data[5] = (data_uint & 0x00ff0000) >> 16;
    send_data.Data[6] = (data_uint & 0x0000ff00) >> 8;
    send_data.Data[7] = data_uint;
  }
  else if (motor_mode == 2)
  {  //速度模式
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x28;  //功能码
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x00;
    uint16_t data_uint = static_cast<uint16_t>(spd_des / 360 * 60 * motor_set.rate);

    send_data.Data[4] = (data_uint & 0xff00) >> 8;  //数据
    send_data.Data[5] = data_uint & 0x00ff;
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
  }
  else if (motor_mode == 3 || motor_mode == 4 || motor_mode == 5 || motor_mode == 6 || motor_mode == 7 ||
           motor_mode == 8)
  {  //力矩模式
    // cout<< "send li"<<endl;
    send_data.Data[0] = 0x00;
    send_data.Data[1] = 0x28;  //功能码
    send_data.Data[2] = 0x00;
    send_data.Data[3] = 0x01;
    uint16_t data_uint = static_cast<uint16_t>(tor_des / force_max * 1000 / motor_set.rate);
    send_data.Data[4] = (data_uint & 0xff00) >> 8;  //数据
    send_data.Data[5] = (data_uint & 0x00ff);
    send_data.Data[6] = 0x00;
    send_data.Data[7] = 0x00;
    // std::cout << "tor is " <<  tor_des/force_max*1000/motor_set.rate
    // <<std::endl;
  }

  // send_data.Data[0] = 0x00;
  // send_data.Data[1] = 0x28;
  // send_data.Data[2] = 0x00;
  // send_data.Data[3] = 0x00;
  // send_data.Data[4] = 0x00;
  // send_data.Data[5] = 0x64;
  // send_data.Data[6] = 0x00;
  // send_data.Data[7] = 0x00;

  // cout << "send id:" << send_data.ID << " "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[0])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[1])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[2])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[3])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[4])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[5])
  // << " "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[6])
  // <<" "
  // 	<< setfill('0') << setw(2) << hex << static_cast<int>(send_data.Data[7])
  // << " "
  // 	<< endl;
}
void CanMotor_ServoClass::MotorRecDecode(VCI_CAN_OBJ rec_data_buff)
{
  // ROS_INFO("=============");
  if (rec_data_buff.Data[1] == 0xCA)
  {
    tor_fdb = -static_cast<double>(static_cast<int16_t>(static_cast<uint16_t>(rec_data_buff.Data[3]) |
                                                       (static_cast<uint16_t>(rec_data_buff.Data[2]) << 8))) /
              1000.0 * force_max * motor_set.rate;

    // cout <<tor_fdb<<endl;
    // ROS_INFO_STREAM("id:" << motor_set.can_id << " tor: " << tor_fdb);
    // ROS_INFO_STREAM("raw data: 0x" << std::hex << static_cast<int16_t>(rec_data_buff.Data[2]) << ",0x"
    //                                << static_cast<int16_t>(rec_data_buff.Data[3]));
    // ROS_INFO_STREAM("recompute:" << std::setprecision(16)
    //                              << static_cast<float>(rec_data_buff.Data[3] | rec_data_buff.Data[2] << 8) / 1000.0 *
    //                                     force_max * motor_set.rate);
    // ROS_INFO_STREAM("raw data: ");
    // for (size_t i = 0; i < sizeof(rec_data_buff.Data) / sizeof(rec_data_buff.Data[0]); ++i)
    // {
    //   ROS_INFO_STREAM("0x" << std::hex << static_cast<int>(rec_data_buff.Data[i]) << " ");
    // }
    // ROS_INFO_STREAM("-------------------");

    pos_fdb = static_cast<double>(static_cast<int32_t>(static_cast<uint32_t>(rec_data_buff.Data[7]) |
                                                       (static_cast<uint32_t>(rec_data_buff.Data[6]) << 8) |
                                                       (static_cast<uint32_t>(rec_data_buff.Data[5]) << 16) |
                                                       (static_cast<uint32_t>(rec_data_buff.Data[4]) << 24))) /
              10000.0 * 360 / motor_set.rate;
    // cout <<pos_fdb<<endl;
    // std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    // std::chrono::duration<double, std::milli> timeElapsed = now - lastCall;
    // lastCall = now;
    // spd_fdb = (pos_fdb - spd_posbuff) * (1000 / timeElapsed.count());  //度每秒;
    // cout <<pos_fdb<<endl;
    // cout <<spd_posbuff<<endl;
    // spd_posbuff = pos_fdb;
    if (spd_fdb > 0)
    {
      tor_fdb = tor_fdb;
    }
  }
  else
  {
    // ROS_INFO("it is cb");
    spd_fdb = static_cast<double>(static_cast<int16_t>(static_cast<uint16_t>(rec_data_buff.Data[2] << 8) |
                                                       (static_cast<uint16_t>(rec_data_buff.Data[3])))) /
              60 * 360 / motor_set.rate;  //度每秒;

    pos_fdb = static_cast<double>(static_cast<int32_t>(static_cast<uint32_t>(rec_data_buff.Data[7]) |
                                                       (static_cast<uint32_t>(rec_data_buff.Data[6]) << 8) |
                                                       (static_cast<uint32_t>(rec_data_buff.Data[5]) << 16) |
                                                       (static_cast<uint32_t>(rec_data_buff.Data[4]) << 24))) /
              10000 * 360 / motor_set.rate;
  }

  // cout <<"========================="<<endl;
  // cout <<now.count()<<endl;

  // cout <<spd_fdb<<endl;

  // cout << "id:" << send_data.ID << " "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[0]) <<" "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[1]) <<" "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[2]) <<" "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[3]) <<" "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[4]) <<" "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[5]) << " "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[6]) <<" "
  // 	<< setfill('0') << setw(2) << hex <<
  // static_cast<int>(rec_data_buff.Data[7]) << " "
  // 	<< endl;
}

uint16_t float_to_uint(float v, float v_min, float v_max, uint32_t width)
{
  float temp;
  int32_t utemp;
  temp = ((v - v_min) / (v_max - v_min)) * ((float)width);
  utemp = (int32_t)temp;
  if (utemp < 0)
    utemp = 0;
  if (utemp > width)
    utemp = width;
  return utemp;
}

float uint_to_float(uint16_t utemp, float v1, float v2)
{
  float v = static_cast<float>((utemp - v1) / v1 * v2);
  // float v = (temp-v1)/v1*v2;

  return v;
}
