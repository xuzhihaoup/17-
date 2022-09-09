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
    //�ɵ���PID ������ʹ�������������ӽ���Ծ����200.0 //
    pidleft.kp = 30;
    pidleft.ki = 5;
    pidleft.kd = 5;
    pidleft.actual_speedmax=7000;
}
int16 pid_left_realise(int16 setspeed,int16 actspeed)//ʵ��pid
{
//    static int16 integrator;
//    pidleft.set_speed = setspeed;//����Ŀ���ٶ�
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
//    pidleft.error_last = pidleft.error_next;//��һ�ε���
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
    //�ɵ���PID ������ʹ�������������ӽ���Ծ����200.0 //
    pidright.kp = 30;
    pidright.ki = 5;
    pidright.kd = 5;
    pidright.actual_speedmax=7000;
}


int16 pid_right_realise(int16 setspeed,int16 actspeed)//ʵ��pid
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
 ����    ����ServoPIDCtrl
 ����    �ܡ����PD����
 ����    ����error���������
 ���� �� ֵ����
 ��ע�����2022/03/16
 ----------------------------------------------------------------*/
uint16 S3010_Right=725;       //��ת
uint16 S3010_Mid  =810;      //�м�
uint16 S3010_Left =895;       //��ת
float ServoKp= 3.5;      //���Kp����ϵ��
float ServoKd= 1;      //���Kd΢��ϵ��
int16 ServoDuty;
float angle=0;
float Motork = 0.85;
void ServoPIDCtrl(int16 error)
{
   static int16 last_error;        //��һ�����

   int16 Out;
   float Out_P;
   float Out_D;
   angle = (float)(Motork *error);

   Out_P = (float)(ServoKp * angle);
   Out_D = (float)(ServoKd * (angle - last_error));

   last_error = angle;

   Out = (int16)(Out_P  + Out_D);

   ServoDuty = S3010_Mid - Out;
   //�޷�
   if(ServoDuty<=S3010_Right)
       ServoDuty = S3010_Right;
   if(ServoDuty>=S3010_Left)
       ServoDuty = S3010_Left;

   pwm_duty(ATOM0_CH1_P33_9,ServoDuty);
}
/*---------------------------------------------------------------
 ����    ����MotorCtr
 ����    �ܡ��������
 ����    ����left_motor_speed�������ٶ�   right_motor_speed���ҵ���ٶ�
 ���� �� ֵ����
 ��ע�����2021/11/15
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
      pwm_duty(ATOM0_CH6_P02_6,   0 );//��ȥ80��Ӳ�����
  }
}

/*---------------------------------------------------------------
 ����    ����dajiao
 ����    �ܡ������ǿ���
 ����    ����deviation����Ƿ���
 ���� �� ֵ����
 ��ע�����2021/11/15
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
 ����    ����error_speed
 ����    �ܡ��ٶȿ���
 ����    ����error_num_speed�����ֵ
 ���� �� ֵ����
 ��ע�����2021/11/15
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


