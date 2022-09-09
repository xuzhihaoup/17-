#include "headfile.h"
#include <motor.h>
#include <image.h>




struct PID pidleft,pidright;

void pid_left_init()
{
    pidleft.set_speed = 0;
    pidleft.actual_speed = 0;
    pidleft.error = 0;
    pidleft.error_next = 0;
    pidleft.error_last = 0;
    //可调节PID 参数。使跟踪曲线慢慢接近阶跃函数200.0 //
    pidleft.kp = 30;
    pidleft.ki = 5;
    pidleft.kd = 5;
    pidleft.actual_speedmax=7000;
}
int16 pid_left_realise(int16 setspeed,int16 actspeed)//实现pid
{
//    static int16 integrator;
//    pidleft.set_speed = setspeed;//设置目标速度
//    pidleft.error = pidleft.set_speed - actspeed;
//
//    integrator += pidleft.error;
//
//    if(integrator>=pidleft.actual_speedmax)
//        integrator = pidleft.actual_speedmax;
//    if(integrator<=-pidleft.actual_speedmax)
//        integrator = -pidleft.actual_speedmax;
//
//    pidleft.actual_speed = (int16)((pidleft.kp *pidleft.error)+(pidleft.kd*(pidleft.error - pidleft.error_next))+(pidleft.ki * integrator));
//
//
//    if(pidleft.actual_speed>pidleft.actual_speedmax)
//        pidleft.actual_speed=pidleft.actual_speedmax;
//    if(pidleft.actual_speed<-pidleft.actual_speedmax)
//        pidleft.actual_speed=-pidleft.actual_speedmax;
//
//    pidleft.error_last = pidleft.error_next;//下一次迭代
//    pidleft.error_next = pidleft.error;
//
//    return pidleft.actual_speed;



    pidleft.error      = setspeed - actspeed;
//    pidleft.error_next = pidleft.error  -  pidleft.error_next;
//    pidleft.error_last = pidleft.error_next - pidleft.error_last;
    pidleft.actual_speed      += pidleft.kp * (pidleft.error - pidleft.error_last) + pidleft.ki * pidleft.error +
            pidleft.kd * (pidleft.error - (2 * pidleft.error_last ) + pidleft.error_next );

    if(pidleft.actual_speed>pidleft.actual_speedmax)
        pidleft.actual_speed=pidleft.actual_speedmax;
    if(pidleft.actual_speed<-pidleft.actual_speedmax)
        pidleft.actual_speed=-pidleft.actual_speedmax;


    pidleft.error_next =  pidleft.error_last;
    pidleft.error_last = pidleft.error;


    return pidleft.actual_speed;
}



void pid_right_init()
{
    pidright.set_speed = 0;
    pidright.actual_speed = 0;
    pidright.error = 0;
    pidright.error_next = 0;
    pidright.error_last = 0;
    //可调节PID 参数。使跟踪曲线慢慢接近阶跃函数200.0 //
    pidright.kp = 30;
    pidright.ki = 5;
    pidright.kd = 5;
    pidright.actual_speedmax=7000;
}


int16 pid_right_realise(int16 setspeed,int16 actspeed)//实现pid
{

    pidright.error      = setspeed - actspeed;
//    pidright.error_next = pidright.error  -  pidright.error_next;
 //   pidright.error_last = pidright.error_next - pidright.error_last;
    pidright.actual_speed     += pidright.kp *(pidright.error -  pidright.error_last) + pidright.ki * pidright.error +
            pidright.kd * (pidright.error - (2 * pidright.error_last ) + pidright.error_next );

    if(pidright.actual_speed>pidright.actual_speedmax)
        pidright.actual_speed=pidright.actual_speedmax;
    if(pidright.actual_speed<-pidright.actual_speedmax)
        pidright.actual_speed=-pidright.actual_speedmax;

    pidright.error_next =  pidright.error_last;
    pidright.error_last = pidright.error;

    return pidright.actual_speed;
}
int16 LeftSpeed,RightSpeed;
int16 LeftMotorPWM,RightMotorPWM;
void MotorCtrl(int16 lefttarget,int16 righttarget)
{
    RightMotorPWM = 0,LeftMotorPWM=0;

    LeftSpeed = gpt12_get(GPT12_T6);
    RightSpeed = -gpt12_get(GPT12_T5);

    gpt12_clear(GPT12_T5);
    gpt12_clear(GPT12_T6);

    LeftMotorPWM = pid_left_realise(lefttarget,LeftSpeed);
    RightMotorPWM = pid_right_realise(righttarget,RightSpeed);

    MotorCtr(LeftMotorPWM,RightMotorPWM);


}
/*---------------------------------------------------------------
 【函    数】ServoPIDCtrl
 【功    能】舵机PD控制
 【参    数】error：中线误差
 【返 回 值】无
 【注意事项】2022/03/16
 ----------------------------------------------------------------*/
uint16 S3010_Right=725;       //右转
uint16 S3010_Mid  =810;      //中间
uint16 S3010_Left =895;       //左转
float ServoKp= 3.5;      //舵机Kp比例系数
float ServoKd= 1;      //舵机Kd微分系数
int16 ServoDuty;
float angle=0;
float Motork = 0.85;
void ServoPIDCtrl(int16 error)
{
   static int16 last_error;        //上一次误差

   int16 Out;
   float Out_P;
   float Out_D;
   angle = (float)(Motork *error);

   Out_P = (float)(ServoKp * angle);
   Out_D = (float)(ServoKd * (angle - last_error));

   last_error = angle;

   Out = (int16)(Out_P  + Out_D);

   ServoDuty = S3010_Mid - Out;
   //限幅
   if(ServoDuty<=S3010_Right)
       ServoDuty = S3010_Right;
   if(ServoDuty>=S3010_Left)
       ServoDuty = S3010_Left;

   pwm_duty(ATOM0_CH1_P33_9,ServoDuty);
}
/*---------------------------------------------------------------
 【函    数】MotorCtr
 【功    能】电机控制
 【参    数】left_motor_speed：左电机速度   right_motor_speed：右电机速度
 【返 回 值】无
 【注意事项】2021/11/15
 ----------------------------------------------------------------*/

void MotorCtr(int32   left_motor_speed, int32   right_motor_speed)
{
  if(left_motor_speed>0)
  {
      pwm_duty(ATOM0_CH5_P02_5,0);
      pwm_duty(ATOM0_CH4_P02_4,left_motor_speed );
  }
  else
  {
      pwm_duty(ATOM0_CH5_P02_5,-left_motor_speed);
      pwm_duty(ATOM0_CH4_P02_4,   0 );
  }
  if(right_motor_speed>0)
  {
      pwm_duty(ATOM0_CH7_P02_7,        0          );
      pwm_duty(ATOM0_CH6_P02_6,  right_motor_speed);
  }
  else
  {
      pwm_duty(ATOM0_CH7_P02_7,-right_motor_speed);
      pwm_duty(ATOM0_CH6_P02_6,   0 );//减去80的硬件误差
  }
}

/*---------------------------------------------------------------
 【函    数】dajiao
 【功    能】舵机打角控制
 【参    数】deviation：打角幅度
 【返 回 值】无
 【注意事项】2021/11/15
 ----------------------------------------------------------------*/
void  dajiao(int16 deviation)
{
   if((0<=deviation&&deviation<5)||(-5<=deviation&&deviation<0))
    {
       pwm_duty(ATOM0_CH2_P21_4,        980-4*deviation );
    }
   else if((-18<=deviation&&deviation<-5)||(5<=deviation&&deviation<=18))
   {
       pwm_duty(ATOM0_CH2_P21_4,        980-3*deviation );
   }
   else
   {
       pwm_duty(ATOM0_CH2_P21_4,        980-80   );
   }
}
/*---------------------------------------------------------------
 【函    数】error_speed
 【功    能】速度控制
 【参    数】error_num_speed：误差值
 【返 回 值】无
 【注意事项】2021/11/15
 ----------------------------------------------------------------*/
void error_speed(int16 error_num_speed)
{
  if(error_num_speed>=7)
  {
   MotorCtr(3000,2100);
  }
  else if(error_num_speed<=-7)
   {
    MotorCtr(2100,3000);
   }
  else if(error_num_speed>=16)
   {
    MotorCtr(3000,2400);
   }
  else if(error_num_speed<=-16)
    {
     MotorCtr(2400,3000);
    }
  else if ((error_num_speed>=0&&error_num_speed<7)||(error_num_speed<0&&error_num_speed>-7))
  {
   MotorCtr(2000,2000);
  }
}


