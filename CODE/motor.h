#ifndef _motor_h
#define _motor_h


extern float angle;


struct PID{
    int16 set_speed;//�趨�ٶ�
    int16 actual_speed;      //ʵ���ٶ�
    int16 actual_speed_last ;//�ϴ��ٶ�
    int16 error;//ƫ��
    int16 error_next;//��һ��ƫ��
    int16 error_last;//����һ��ƫ��
    float kp,ki,kd;//������������֣�΢�ֲ���
    int16 actual_speedmax;

};
void MotorCtrl(int16 lefttarget,int16 righttarget);
void pid_left_init();//PID������ʼ��
int16 pid_left_realise(int16 setspeed,int16 actspeed);//ʵ��PID�㷨
void pid_right_init();//PID������ʼ��
int16 pid_right_realise(int16 setspeed,int16 actspeed);//ʵ��PID�㷨
void ServoPIDCtrl(int16 error);
void MotorCtr(int32  left_motor_speed,  int32  right_motor_speed);
void  dajiao(int16 deviation);
void error_speed(int16 error_num);
#endif
