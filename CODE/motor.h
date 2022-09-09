#ifndef _motor_h
#define _motor_h


extern float angle;


struct PID{
    int16 set_speed;//设定速度
    int16 actual_speed;      //实际速度
    int16 actual_speed_last ;//上次速度
    int16 error;//偏差
    int16 error_next;//上一个偏差
    int16 error_last;//上上一个偏差
    float kp,ki,kd;//定义比例，积分，微分参数
    int16 actual_speedmax;

};
void MotorCtrl(int16 lefttarget,int16 righttarget);
void pid_left_init();//PID参数初始化
int16 pid_left_realise(int16 setspeed,int16 actspeed);//实现PID算法
void pid_right_init();//PID参数初始化
int16 pid_right_realise(int16 setspeed,int16 actspeed);//实现PID算法
void ServoPIDCtrl(int16 error);
void MotorCtr(int32  left_motor_speed,  int32  right_motor_speed);
void  dajiao(int16 deviation);
void error_speed(int16 error_num);
#endif
