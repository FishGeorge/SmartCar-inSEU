/*********************************************************
@2017CameraDemo
@�̼��⣺����V2.4
@author��wgq & lq
@2017.11.27
@for seu2017 ����ͷ��
*********************************************************/

#include "chlib_k.h"
#include "varieble.h"
#include "img_processing.h"
#include "oled.h"
#include "init.h"

void PID_init();
float PID_Motor_computing(int);
float PID_Steer_computing(float);
void Send_image();
void Send_condition();
int GetDir_err();
bool PreWarningCurveRoad(int);
void PTB11_EXTI_ISR();
void PIT_Speed_ISR();
void dispcondition(int);
void brake(int,int,int);
int AbsoluteValue(int);

// pid����
#define T    0.01     // ��������
// �������
#define Kp   7.5        // PID���ڵı�������
#define Ti   0.00     // PID���ڵĻ��ֳ���
#define Td   0.00     // PID���ڵ�΢��ʱ�䳣��
#define S_Kpp Kp
#define S_Ki Kp * T / Ti// ����Ki
#define S_Kd Kp * Td / T
// ���ķ�ֵ��С�������ֵ��ʱ�򣬲���PID��������������СʱƵ������������
#define Emin 0
// �������
#define MKp   0.3        // PID���ڵı�������
#define MTi   0.04     // PID���ڵĻ��ֳ���
#define MTd   0.00     // PID���ڵ�΢��ʱ�䳣��
#define M_Kp MKp
#define M_Ki MKp * T / MTi
#define M_Kd MKp * MTd / T// ����Kd

struct _pid{  
  float SetSpeed; // �����趨ֵ  
  float ActualSpeed; // ����ʵ��ֵ  
  float err; // ����ƫ��ֵ  
  float err_l; // ������һ��ƫ��ֵ  
  float err_ll; // ��������ǰ��ƫ��ֵ  
  float pidKp,pidKi,pidKd; // ������������֡�΢��ϵ��  
}pid_s,pid_m;

// ��־λ
int mark_loop=0;
bool mark_motor=false;
bool mark_uart=false;
bool mark_set=false;
bool onStraightRoad=true;
bool comingCurveRoad=false;
bool comingTerminal=false;

//�Ҷ�ֵ
int WB=190;

// Dir_err�жϷ�Χ
int imageUB=15;
int imageLB=40;

// ��������
int StraightSpeed=90;
int CurveSpeed=90;
int MotorFrequency=0;
int initFrequencyLevel=1500;
int MAXFrequency=3000;
int cPulse=0;
int cSpeed=0;//cm/s
int cPreDirErr=0;
int cDirErr=0;
int S_PWM=725;
int S_center=725;
  //64����ֵ725
  //185����ֵ710

// pit������������ us
int pitTime=10000;
// ֱ���ж�Dirƫ����
int StraightR=10;
// ���ǰհ��
int PreCurveR=8;
// ������������
int uart_n=0;
int uart_count=100;

int main(){
  // ��ʼ��֮ǰ�ȹص������ж�
  DisableInterrupts;
  
  // OLED��ʼ��
  OLED_Init();
  OLED_Clear();
  // UART��ʼ��
  UART_QuickInit(UART3_RX_PC16_TX_PC17,115200);
  // PWT��ʱ����ʼ��
  DelayInit();
  // ����ͷ��ʼ��
  init_ov7620();
  // PID�㷨��ʼ��
  PID_init();
  // �����ʼ��
  FTM_PWM_QuickInit(FTM2_CH0_PB18,kPWM_EdgeAligned,50,0);
  // �����ʼ��
  // 5_05 > 7_07 ahead
  // 5_05 < 7_07 back
  FTM_PWM_QuickInit(FTM0_CH5_PD05,kPWM_EdgeAligned,10000,0);
  FTM_PWM_QuickInit(FTM0_CH7_PD07,kPWM_EdgeAligned,10000,0);
  // ��������ʼ��
  // ftm�����������ʼ��
  FTM_QD_QuickInit(FTM1_QD_PHA_PB00_PHB_PB01, kFTM_QD_NormalPolarity,kQD_CountDirectionEncoding);
  // pit�жϼ�ʱ����ʼ��/ ��λus /100msһ����������
  PIT_QuickInit(HW_PIT_CH0,pitTime);
  // pit�жϼ��䴦����ע��
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);
  PIT_CallbackInstall(HW_PIT_CH0, PIT_Speed_ISR);
  // GPIO��ʼ��
  // ���뿪��/��������
  GPIO_QuickInit(HW_GPIOB, 20, kGPIO_Mode_IPD);
  GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IPD);
  GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IPD);
  GPIO_QuickInit(HW_GPIOB, 23, kGPIO_Mode_IPD);
  // ����KEY
  // buttun1/�������ж�
  GPIO_QuickInit(HW_GPIOB, 11, kGPIO_Mode_IFT);
  GPIO_ITDMAConfig(HW_GPIOB, 11, kGPIO_IT_RisingEdge, true);
  GPIO_CallbackInstall(HW_GPIOB,PTB11_EXTI_ISR);
  // buttun2/��������
  GPIO_QuickInit(HW_GPIOB, 17, kGPIO_Mode_IPD);
  // LED��
  // ���LED7/�������
  GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
  // ���LED8/�������
  GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_OPP);
  
  // �������
  // <725 turn left
  // >725 turn right
  // Effective range: 600~850
  FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,S_center);
  //64����ֵ725
  //185����ֵ710
  
  // ��ʼ��������ϣ�ʹ���ж�
  EnableInterrupts;
  // �������ѭ������LED7��
  PEout(25)=1;
  
  while(1){
    // ��10ms
    DelayMs(6);
    
    // ��ͬ���������ı�
    // ����1
    if(PBin(22)){
      
    }
    
    // Ѱ�ߵ�Lx Rx�����̰����˵�������ͷ��ȡͼ��
    searchline_OV7620(WB);
    // ���ڶ��
    //64����ֵ725
    //185����ֵ710
    S_PWM+=(int)PID_Steer_computing(GetDir_err());
//    S_PWM=S_center+(int)PID_Steer_computing(GetDir_err());
    if(S_PWM>S_center+130)S_PWM=S_center+130;
    if(S_PWM<S_center-130)S_PWM=S_center-130;
    // ����2
    if(PBin(23))FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,S_PWM);
    // ����3/��ʾѰ�߽��ʾ��ͼ��OLED��
    if(PBin(20)){if(!PBin(17))dispimage(mark_loop,cDirErr,cPreDirErr,cSpeed,MotorFrequency);else dispcondition(mark_loop);mark_loop++;}
    else OLED_Clear();
    // ����4/ͨ�����ڷ���data����λ�� LED8��ʾ����״̬
    if(PBin(21)){
      if(mark_uart){PCout(18)=1;Send_image();PCout(18)=0;}
      else{uart_n++;if(uart_n%uart_count==0){PCout(18)=1;Send_condition();PCout(18)=0;}}
    }
    // button2�л�mark_uart
    if(PBin(17))
      if(!mark_uart)mark_uart=true;
      else mark_uart=false;
    
    // ֱ����ж�
    if(cDirErr>=-StraightR&&cDirErr<=StraightR) onStraightRoad=true;
    else onStraightRoad=false;
    // ǰհ����ж�
    if(onStraightRoad)comingCurveRoad=PreWarningCurveRoad(PreCurveR);
    // �յ��ж�
//    if((Lx[10]-Rx[10])<30&&(Lx[15]-Rx[15])<30)comingTerminal=true;
    if((Lx[35]-Rx[35])<30&&(Lx[40]-Rx[40])<35)comingTerminal=true;
    else comingTerminal=false;
    
    // ǰհ�յ㣬�رյ��
    if(comingTerminal){
      mark_motor=false;
//      pid_m.SetSpeed=0;
    }
    // ��ֱ����ǰհֱ��
    else if(onStraightRoad&&!comingCurveRoad) pid_m.SetSpeed=StraightSpeed;
    // ��ֱ����ǰհ���
    else if(onStraightRoad&&comingCurveRoad) pid_m.SetSpeed=CurveSpeed;
    // �����
    else{
      pid_m.SetSpeed=CurveSpeed;
      onStraightRoad=false;
    }
    
    // ������ʿ��� �ջ�����
    if(mark_motor)
      MotorFrequency+=(int)PID_Motor_computing(cSpeed);
    else 
      MotorFrequency=0;
    // ����pwm�����Сռ�ձ�����
    if(MotorFrequency>MAXFrequency) MotorFrequency=MAXFrequency;
    if(MotorFrequency<-initFrequencyLevel) MotorFrequency=-initFrequencyLevel;
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,MotorFrequency+initFrequencyLevel);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,initFrequencyLevel);
  }
}

//  ͼ����ó�����ƫ��ֵint
int GetDir_err(){
  int err_out=0;
  int err_sum=0;
  int k=0;
  // Range:0-49
  // 15-35��
  // 12.14  20-45��
  // �����ʣ�����15-35
  // 3.5 10-45 �ֶ�ϵ��
  // 15-40
//  int imageUB=15;
//  int imageLB=40;
  for(int i=imageUB;i<=imageLB;i++){
    if(0<Cx[i]<152){
      if(i<=30)
        err_sum+=1*(Cx[i]-76);
      else
        err_sum+=1.5*(Cx[i]-76);
    }else
      k++;
  }
  if(k==(imageLB-imageUB+1)) err_out=0;
  else err_out=err_sum/(imageLB-imageUB+1-k);
  cDirErr=err_out+2;
//  cDirErr=err_out;
  // ͼ������һ�����
  return cDirErr;
}

//  ���Ԥ��
bool PreWarningCurveRoad(int CurveLimit){
  int err_out=0;
  int err_sum=0;
  int k=0;
  for(int i=0;i<=imageUB;i++){
    if(0<Cx[i]<152)
        err_sum+=1*(Cx[i]-76);
    else
      k++;
  }
  if(k==(imageUB-0+1)) err_out=0;
  else err_out=err_sum/(imageUB-0+1-k);
  cPreDirErr=err_out;
  if(err_out>-CurveLimit&&err_out<CurveLimit)return false;
  else return true;
}

// PID�㷨��ʼ��ʵ��
void PID_init(){
  // ����ò���
  pid_s.err=0.0;  
  pid_s.err_ll=0.0;  
  pid_s.err_l=0.0; 
  
  pid_s.pidKp=S_Kpp;
  pid_s.pidKi=S_Ki;
  pid_s.pidKd=S_Kd;
  
  // ����ò���
  pid_m.SetSpeed=StraightSpeed;  
  pid_m.ActualSpeed=0.0;  
  pid_m.err=0.0;  
  pid_m.err_ll=0.0;  
  pid_m.err_l=0.0; 
  
  pid_m.pidKp=M_Kp;
  pid_m.pidKi=M_Ki;
  pid_m.pidKd=M_Kd; 
}  

// ���pid�㷨_pd����ʽ/�Է���ʽ
float PID_Steer_computing(float err){
//  pid_s.err=err; 
//  
//  float increment
//    =pid_s.Kp*(pid_s.err-pid_s.err_l)+pid_s.Kd*(pid_s.err-2*pid_s.err_l+pid_s.err_ll);
////  float increment
////    =pid_s.Kp*pid_s.err-pid_s.Kd*pid_s.err_l;
//  
//  pid_s.err_ll=pid_s.err_l;
//  pid_s.err_l=pid_s.err;
//  
//  return increment;
  
  float increment=0;
  pid_s.err = err;// ��ֵ���� 
  if(AbsoluteValue((int)pid_s.err) < Emin )// ���ķ�ֵ(��������?) 
    increment = 0;
  else{
    increment = pid_s.pidKp*(pid_s.err-pid_s.err_l) + pid_s.pidKd*(pid_s.err-2*pid_s.err_l+pid_s.err_ll);
    pid_s.err_ll = pid_s.err_l;
    pid_s.err_l = pid_s.err;
  }
  // PWM���ֵ�޷�����main()��
  return increment;
}

// ���pid�㷨_pi����ʽ
float PID_Motor_computing(int speed){
  pid_m.ActualSpeed=speed;
  pid_m.err=pid_m.SetSpeed-pid_m.ActualSpeed; 
  
  float incrementFrequency
    =pid_m.pidKp*(pid_m.err-pid_m.err_l)+pid_m.pidKi*pid_m.err;
  
  pid_m.err_ll=pid_m.err_l;
  pid_m.err_l=pid_m.err;
  
  return incrementFrequency;
}

// UART�˿ڷ�������ͷͼ��ʵ��
void Send_image(){
  UART_WriteByte(HW_UART3,0X01);
  UART_WriteByte(HW_UART3,0XFE);
  
  uint32_t i,j;
  for(i=0;i<row_num;i++){
    for(j=0;j<col_num;j++){
      if(imgadd[i*col_num+j]==0xFF)
        UART_WriteByte(HW_UART3,imgadd[i*col_num+j]-1);
      else
        UART_WriteByte(HW_UART3,imgadd[i*col_num+j]);
    }
  }
  
  UART_WriteByte(HW_UART3,0XFE);
  UART_WriteByte(HW_UART3,0X01);
}

void Send_condition(){
//  UART_WriteByte(HW_UART3,'t');
  char num_arr1[4];
//  num_arr1[0] = S_PWM / 1000+48;
//  num_arr1[1] = S_PWM / 100+48;
//  num_arr1[2] = S_PWM / 10 % 10+48;
//  num_arr1[3] = S_PWM % 10+48;
//  UART_printf(HW_UART3,"S_PWM= ");
//  UART_printf(HW_UART3,num_arr1);
  
  int Err=0;
  if(cPreDirErr<0)Err=-cPreDirErr;
  else Err=cPreDirErr;
  num_arr1[0] = Err / 1000+48;
  num_arr1[1] = Err / 100+48;
  num_arr1[2] = Err / 10 % 10+48;
  num_arr1[3] = Err % 10+48;
  UART_printf(HW_UART3,"  PE= ");
  UART_printf(HW_UART3,num_arr1);
  
  if(cDirErr<0)Err=-cDirErr;
  else Err=cDirErr;
  num_arr1[0] = Err / 1000+48;
  num_arr1[1] = Err / 100+48;
  num_arr1[2] = Err / 10 % 10+48;
  num_arr1[3] = Err % 10+48;
  UART_printf(HW_UART3,"  Err= ");
  UART_printf(HW_UART3,num_arr1);
  
  num_arr1[0] = MotorFrequency / 1000+48;
  num_arr1[1] = MotorFrequency / 100 % 10+48;
  num_arr1[2] = MotorFrequency / 10 % 10+48;
  num_arr1[3] = MotorFrequency % 10+48;
  UART_printf(HW_UART3,"  PWM= ");
  UART_printf(HW_UART3,num_arr1);
  
  num_arr1[0] = '0';
  num_arr1[1] = cSpeed / 100+48;
  num_arr1[2] = cSpeed / 10 % 10+48;
  num_arr1[3] = cSpeed % 10+48;
  UART_printf(HW_UART3,"  Speed= ");
  UART_printf(HW_UART3,num_arr1);
  
  UART_WriteByte(HW_UART3,'\n');
}

// buttun1�жϴ�����
/*
�߼���
buttun1����ת���������
*/
void PTB11_EXTI_ISR(){
    if(!mark_motor)mark_motor=true;
    else mark_motor=false;
}

// PIT��ʱ�жϴ����� ���ڱ������������
void PIT_Speed_ISR(){
  int16_t val;
  uint8_t dir;
  FTM_QD_GetData(HW_FTM1, &val, &dir);
  cPulse=-(int)val;
  cSpeed=(int)((float)cPulse/(float)512*(float)40/(float)105*17.3*1000000/pitTime);
  FTM_QD_ClearCount(HW_FTM1);
}

void dispcondition(int n){
  // �����һ����OLED��ʾͼ�񣨿��ܵ���Ƶ����
  if(!(n%3))// ÿ3������clearһ�Σ�����Ƶ��
    OLED_Clear();
  
  // pid������ʾ
  OLED_ShowString_1206(0,0,"s_Kp=",1);
  OLED_ShowNum_1206(30,0,pid_s.pidKp,1);
  
  OLED_ShowString_1206(0,13,"s_Ki=",1);
  OLED_ShowNum_1206(30,13,pid_s.pidKi,1);
  
  OLED_ShowString_1206(0,26,"s_Kd=",1);
  OLED_ShowNum_1206(30,26,pid_s.pidKd,1);
  
  OLED_ShowString_1206(64,0,"m_Kp=",1);
  OLED_ShowNum_1206(94,0,pid_m.pidKp,1);
  
  OLED_ShowString_1206(64,13,"m_Ki=",1);
  OLED_ShowNum_1206(94,13,pid_m.pidKi,1);
  
  OLED_ShowString_1206(64,26,"m_Kd=",1);
  OLED_ShowNum_1206(94,26,pid_m.pidKd,1);
  
  OLED_ShowString_1206(0,39,"PreDirErr=",1);
  char num_arr[3];
  num_arr[0] = cPreDirErr / 100+48;
  num_arr[1] = cPreDirErr / 10 % 10+48;
  num_arr[2] = cPreDirErr % 10+48;
  OLED_ShowString_1206(66,39,num_arr,1);
  
  // ˢ���Դ棬��ʾͼ��
  OLED_Refresh_Gram();
}

// ɲ��
// 5_05 > 7_07 ahead
// 5_05 < 7_07 back
void brake(int tier,int duty,int time){
  PCout(18)=1;
  int err1=(MotorFrequency-duty)/2;
  int err2=duty/2;
  if(err1<err2)err1=err2;
  if(tier==0){
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,0);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,0);
    DelayMs(time);
  }else if(tier==1){
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,pid_m.pidKp*duty);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,0);
    DelayMs(time);
  }else if(tier==2){
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,0);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,(int)(duty));
    DelayMs(time);
  }
  PCout(18)=0;
}

// int����ֵ���
int AbsoluteValue(int v){
  if(v<0)v=-v;
  return v;
}