/*
@2017CameraDemo
@固件库：超核V2.4
@author：wgq & lq
@2017.11.27
@for seu2017 摄像头组
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
void dispcondition();
void brake(int,int,int);
int AbsoluteValue(int);

// pid参数
float T=    0.02;     // 采样周期
// 舵机部分
float Kp=   7.2;        // PID调节的比例常数
float Ti=   1;//不用// PID调节的积分常数
float Td=   1.1;     // PID调节的微分时间常数
//#define S_Kpp= Kp
//#define S_Ki= Kp * T / Ti// 不用Ki
//#define S_Kd= Kp * Td / T
// 除骤变
float EEmax=100;
// 误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
float Emin= 0;
// 电机部分
float MKp=   3;        // PID调节的比例常数
float MTi=   0.05;     // PID调节的积分常数
float MTd=   1;//不用// PID调节的微分时间常数
//#define M_Kp= MKp
//#define M_Ki= MKp * T / MTi
//#define M_Kd= MKp * MTd / T// 不用Kd

struct _pid{  
  float SetSpeed; // 定义设定值  
  float ActualSpeed; // 定义实际值  
  float err; // 定义偏差值  
  float err_l; // 定义上一个偏差值  
  float err_ll; // 定义最上前的偏差值  
  float pidKp,pidKi,pidKd; // 定义比例、积分、微分系数  
}pid_s,pid_m;

// 标志位
int mark_loop=0;
bool mark_motor=false;
bool mark_uart=false;
bool mark_afterset=false;
bool onStraightRoad=true;
bool comingCurveRoad=false;
int comingTerminal=0;

// 调参模式参数设置
int mark_set=1;
//1-直道车速，2-弯道车速
//3-S_Kp，4-S_Kd
//5-M_Kp，6-M_Ki
//7-mark_setting_amount
int mark_setting_amount=2;
float setting_amount[5]={0.01,0.05,0.5,1,5.0};

//灰度值
int WB=175;

// Dir_err判断范围
int imageUB=0;
int imageLB=45;

// 车况参数
int StraightSpeed=130;
int CurveSpeed=110;
int MotorFrequency=0;
int initFrequencyLevel=1500;
int MAXFrequency=3500;
int cPulse=0;
int cSpeed=0;//cm/s
int cPreDirErr=0;
int cDirErr=0;
int S_PWM=725;
int S_center=725;
//64车中值725
//185车中值710

// pit采样脉冲周期 us
int pitTime=10000;
// 直道判定Dir偏差限
int StraightR=8;
// 弯道前瞻限
int PreCurveR=10;
// 发送周期限制
int uart_n=0;
int uart_count=100;

int main(){
  // 初始化之前先关掉所有中断
  DisableInterrupts;
  
  // OLED初始化
  OLED_Init();
  OLED_Clear();
  // UART初始化
  UART_QuickInit(UART3_RX_PC16_TX_PC17,115200);
  // PWT定时器初始化
  DelayInit();
  // 摄像头初始化
  init_ov7620();
  // PID算法初始化
  PID_init();
  // 舵机初始化
  FTM_PWM_QuickInit(FTM2_CH0_PB18,kPWM_EdgeAligned,50,0);
  // 电机初始化
  // 5_05 > 7_07 ahead
  // 5_05 < 7_07 back
  FTM_PWM_QuickInit(FTM0_CH5_PD05,kPWM_EdgeAligned,10000,0);
  FTM_PWM_QuickInit(FTM0_CH7_PD07,kPWM_EdgeAligned,10000,0);
  // 编码器初始化
  // ftm脉冲计数器初始化
  FTM_QD_QuickInit(FTM1_QD_PHA_PB00_PHB_PB01, kFTM_QD_NormalPolarity,kQD_CountDirectionEncoding);
  // pit中断计时器初始化/ 单位us /100ms一个采样周期
  PIT_QuickInit(HW_PIT_CH0,pitTime);
  // pit中断及其处理函数注册
  PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);
  PIT_CallbackInstall(HW_PIT_CH0, PIT_Speed_ISR);
  // GPIO初始化
  // 拨码开关/下拉输入
  GPIO_QuickInit(HW_GPIOB, 20, kGPIO_Mode_IPD);
  GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IPD);
  GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IPD);
  GPIO_QuickInit(HW_GPIOB, 23, kGPIO_Mode_IPD);
  // 按键KEY
  // Key1/上升沿中断
  GPIO_QuickInit(HW_GPIOB, 11, kGPIO_Mode_IFT);
  GPIO_ITDMAConfig(HW_GPIOB, 11, kGPIO_IT_RisingEdge, true);
  GPIO_CallbackInstall(HW_GPIOB,PTB11_EXTI_ISR);
  // Key2/下拉输入
  GPIO_QuickInit(HW_GPIOB, 17, kGPIO_Mode_IPD);
  // LED灯
  // 版标LED7/推挽输出
  GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
  // 版标LED8/推挽输出
  GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_OPP);
  
  // 舵机摆正
  // <725 turn left
  // >725 turn right
  // Effective range: 600~850
  FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,S_center);
  //64车中值725
  //185车中值710
  
  // 初始化设置完毕，使能中断
  EnableInterrupts;
  // 程序进入循环，亮LED7灯
  PEout(25)=1;
  
  while(1){
    // 凑20ms计算周期
    DelayMs(16);
    
    /*
    拨码开关及按键Key使用说明：
    1、主模式调整：
     开关1打开，调参模式
     开关1关闭，车道模式
    2、调参模式下：（OLED屏显示设定车速及PID参数）
     开关4打开时，
       按Key1/Key2切换准备调整的参数（OLED屏上突白表示选中）
     开关4关闭时，
       按Key1加选中参数，按Key2减选中参数。
     开关2无用
     开关3无用
    3、车道模式下：（OLED屏显示图像及主要参数）
     开关2控制舵机
     开关3控制OLED屏
     开关4控制UART发送
     按击Key1控制电机
     按击Key2控制UART发送内容
    */
    // 调参模式逻辑部分（Key2部分）
    // 开关1打开
    if(PBin(22)){
      // 调参模式关闭电机
      mark_motor=false;
      // 
      mark_afterset=true;
      // 调参模式下Key2控制逻辑
      // 开关4打开时
      if(PBin(21)){
        // 按Key2
        if(PBin(17)&&mark_loop%20==0){
          mark_set--;
          if(mark_set==0)mark_set=7;
        }
      }
      // 开关4关闭时
      else{
        // 按Key2
        if(PBin(17)&&mark_loop%20==0){
          if(mark_set==1) StraightSpeed-=setting_amount[mark_setting_amount];
          else if(mark_set==2) CurveSpeed-=setting_amount[mark_setting_amount];
          else if(mark_set==3) Kp-=setting_amount[mark_setting_amount];
          else if(mark_set==4) Td-=setting_amount[mark_setting_amount];
          else if(mark_set==5) MKp-=setting_amount[mark_setting_amount];
          else if(mark_set==6) MTi-=setting_amount[mark_setting_amount];
          else if(mark_set==7){
            mark_setting_amount--;
            if(mark_setting_amount==-1)mark_setting_amount=4;
          }
        }
      }
      mark_loop++;
      if(mark_loop==10000)mark_loop=-1;
    }
    else if(mark_afterset){
      // PID算法初始化
      PID_init();
      mark_afterset=false;
    }
    
    // 寻线到Lx Rx，过程包括了调用摄像头获取图像
    searchline_OV7620(WB);
    // 调节舵机
    //64车中值725
    //185车中值710
    S_PWM=725+(int)PID_Steer_computing(GetDir_err());
//    S_PWM+=(int)PID_Steer_computing(GetDir_err());
    if(S_PWM>S_center+140)S_PWM=S_center+140;
    if(S_PWM<S_center-140)S_PWM=S_center-140;
    // 车道模式开关2 控制舵机开关
    if(!PBin(22)&&PBin(23))FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,S_PWM);
    // 车道模式开关3 控制OLED屏开关 /调速模式下OLED屏常开
    // 车道模式下显示图像及主要参数
    if(!PBin(22)&&PBin(20)) dispimage(mark_loop,cDirErr,cPreDirErr,cSpeed,MotorFrequency);
    // 调参模式下显示设定车速及PID参数
    else if(PBin(22)) dispcondition();
    // 车道模式开关4 控制串口开关 通过串口发送data到上位机 LED8显示发送状态
    if(PBin(21))
      if(mark_uart){PCout(18)=1;Send_image();PCout(18)=0;}
      else{uart_n++;if(uart_n%uart_count==0){PCout(18)=1;Send_condition();PCout(18)=0;}}
    // 车道模式下按键Key2切换mark_uart以切换发送内容
    if(!PBin(22)&&PBin(17))
      if(!mark_uart)mark_uart=true;
      else mark_uart=false;
      
      // 直弯道判断
      if(cDirErr>=-StraightR&&cDirErr<=StraightR) onStraightRoad=true;
      else onStraightRoad=false;
      // 前瞻弯道判断
      if(onStraightRoad)comingCurveRoad=PreWarningCurveRoad(PreCurveR);
      // 终点判断
      //    if((Lx[10]-Rx[10])<30&&(Lx[15]-Rx[15])<30)comingTerminal=true;
//      else comingTerminal=false;
      if((Lx[35]-Rx[35])<30&&(Lx[40]-Rx[40])<35)comingTerminal++;
      
      // 前瞻终点，关闭电机
      if(comingTerminal==2){
        mark_motor=false;
        //      pid_m.SetSpeed=0;
      }
      // 在直道且前瞻直道
      else if(onStraightRoad&&!comingCurveRoad) pid_m.SetSpeed=StraightSpeed;
      // 在直道而前瞻弯道
      else if(onStraightRoad&&comingCurveRoad) pid_m.SetSpeed=CurveSpeed;
      // 在弯道
      else{
        pid_m.SetSpeed=CurveSpeed;
        onStraightRoad=false;
      }
      
      // 电机功率控制 闭环控速
      if(mark_motor)
        MotorFrequency+=(int)PID_Motor_computing(cSpeed);
      else 
        MotorFrequency=0;
      // 正极pwm最大最小占空比限制
      if(MotorFrequency>MAXFrequency) MotorFrequency=MAXFrequency;
      if(MotorFrequency<-initFrequencyLevel) MotorFrequency=-initFrequencyLevel;
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,MotorFrequency+initFrequencyLevel);
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,initFrequencyLevel);
  }
}

//  图像处理得出方向偏差值int
int GetDir_err(){
  int err_out=0;
  int err_sum=0;
  int k=0;
  // Range:0-49
  // 15-35行
  // 12.14  20-45行
  // 不合适，暂用15-35
  // 3.5 10-45 分段系数
  // 15-40
  //  int imageUB=15;
  //  int imageLB=40;
  for(int i=imageUB;i<=imageLB;i++){
    if(Cx[i]!=76){// 十字路口区分
      err_sum+=(Cx[i]-76)*260/(210+i);//图像矫正
    }else
      k++;
  }
  if(k==(imageLB-imageUB+1)) err_out=0;
  else err_out=err_sum/(imageLB-imageUB+1-k);
  
//  if(err_sum<=10){
//    imageUB=0;
//    imageLB=10;
//    for(int i=imageUB;i<=imageLB;i++){
//      if(78<Cx[i]||Cx[i]<74){
//        err_sum+=(Cx[i]-76)*260/(210+i);//图像矫正
//      }else
//        k++;
//    }
//    if(k==(imageLB-imageUB+1)) err_out=0;
//    else err_out=err_sum/(imageLB-imageUB+1-k);
//  }
//  cDirErr=err_out+5;
    cDirErr=err_out;
  // 图像处理有一点误差
  return cDirErr;
}

//  弯道预警
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

// PID算法初始化实现
void PID_init(){
  // 舵机用参数
  pid_s.err=0.0;  
  pid_s.err_ll=0.0;  
  pid_s.err_l=0.0; 
  
  pid_s.pidKp=Kp;
  pid_s.pidKi=Kp * T / Ti;
  pid_s.pidKd=Kp * Td / T;
  
  // 电机用参数
  pid_m.SetSpeed=StraightSpeed;  
  pid_m.ActualSpeed=0.0;  
  pid_m.err=0.0;  
  pid_m.err_ll=0.0;  
  pid_m.err_l=0.0; 
  
  pid_m.pidKp=MKp;
  pid_m.pidKi=MKp * T / MTi;
  pid_m.pidKd=MKp * MTd / T; 
}  

// 舵机pid算法_pd自发明式/增量式
float PID_Steer_computing(float err){
  float increment=0;
  pid_s.err = err;// 差值运算 
  if(AbsoluteValue((int)(pid_s.err-pid_s.err_l)) > EEmax )// 除骤变 
    increment = 0;
  if(AbsoluteValue((int)pid_s.err) < Emin )// 误差的阀值(死区控制?) 
    increment = 0;
  else{
    increment=Kp*pid_s.err-Td*pid_s.err_l;
//    increment = pid_s.pidKp*(pid_s.err-pid_s.err_l) + pid_s.pidKd*(pid_s.err-2*pid_s.err_l+pid_s.err_ll);
    pid_s.err_ll = pid_s.err_l;
    pid_s.err_l = pid_s.err;
  }
  // PWM输出值限幅放在main()里
  return increment;
}

// 电机pid算法_pi增量式
float PID_Motor_computing(int speed){
  pid_m.ActualSpeed=speed;
  pid_m.err=pid_m.SetSpeed-pid_m.ActualSpeed; 
  
  float incrementFrequency
    =pid_m.pidKp*(pid_m.err-pid_m.err_l)+pid_m.pidKi*pid_m.err;
  
  pid_m.err_ll=pid_m.err_l;
  pid_m.err_l=pid_m.err;
  
  return incrementFrequency;
}

// UART端口发送摄像头图像实现
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

// Key1中断处理函数
/*
逻辑：
     开关1打开，调参模式
       开关4打开时，
         按Key1/Key2切换准备调整的参数（OLED屏上突白表示选中）
       开关4关闭时，
         按Key1加选中参数
     开关1关闭，车道模式
       按Key1控制电机
*/
void PTB11_EXTI_ISR(){
  // 开关1关闭时，车道模式
  if(!PBin(22)){
    if(!mark_motor)mark_motor=true;
    else mark_motor=false;
  }
  // 开关1打开时，调参模式
  else{
    // 开关4打开时
    if(PBin(21)){
      mark_set++;
      if(mark_set==8)mark_set=1;
    }
    // 开关4关闭时
    else{
      if(mark_set==1) StraightSpeed+=setting_amount[mark_setting_amount];
      else if(mark_set==2) CurveSpeed+=setting_amount[mark_setting_amount];
      else if(mark_set==3) Kp+=setting_amount[mark_setting_amount];
      else if(mark_set==4) Td+=setting_amount[mark_setting_amount];
      else if(mark_set==5) MKp+=setting_amount[mark_setting_amount];
      else if(mark_set==6) MTi+=setting_amount[mark_setting_amount];
      else if(mark_set==7){
        mark_setting_amount++;
        if(mark_setting_amount==5)mark_setting_amount=0;
      }
    }
  }
}

// PIT定时中断处理函数 用于编码器脉冲计数
void PIT_Speed_ISR(){
  int16_t val;
  uint8_t dir;
  FTM_QD_GetData(HW_FTM1, &val, &dir);
  cPulse=-(int)val;
  cSpeed=(int)((float)cPulse/(float)512*(float)40/(float)105*17.3*1000000/pitTime);
  FTM_QD_ClearCount(HW_FTM1);
}

void dispcondition(){
  // 清除上一周期OLED显示图像（可能导致频闪）
  //  if(!(n%3))// 每3个周期clear一次，降低频闪
  OLED_Clear();
  // 设定车速显示
  OLED_ShowString_1206(0,0,"S_S=",1);
  if(mark_set==1)
    OLED_ShowNum_1206(24,0,StraightSpeed,0);
  else
    OLED_ShowNum_1206(24,0,StraightSpeed,1);
  OLED_ShowString_1206(64,0,"C_S=",1);
  if(mark_set==2)
    OLED_ShowNum_1206(88,0,CurveSpeed,0);
  else
    OLED_ShowNum_1206(88,0,CurveSpeed,1);
  
  // PID参数显示
  // 舵机部分
  OLED_ShowString_1206(0,13,"s_Kp=",1);
  if(mark_set==3)
    OLED_ShowNum_1206(30,13,Kp+(float)10,0);
  else
    OLED_ShowNum_1206(30,13,Kp+(float)10,1);
  OLED_ShowString_1206(0,26,"s_Td=",1);
  if(mark_set==4)
    OLED_ShowNum_1206(30,26,Td+(float)10,0);
  else
    OLED_ShowNum_1206(30,26,Td+(float)10,1);
  // 电机部分
  OLED_ShowString_1206(64,13,"m_Kp=",1);
  if(mark_set==5)
    OLED_ShowNum_1206(94,13,MKp+(float)10,0);
  else
    OLED_ShowNum_1206(94,13,MKp+(float)10,1);
  OLED_ShowString_1206(64,26,"m_Ti=",1);
  if(mark_set==6)
    OLED_ShowNum_1206(94,26,MTi+(float)10,0);
  else
    OLED_ShowNum_1206(94,26,MTi+(float)10,1);
  
  // 调整幅度
  OLED_ShowString_1206(0,45,"set_a=",1);
  if(mark_set==7)
    OLED_ShowNum_1206(36,45,setting_amount[mark_setting_amount]+(float)10,0);
  else
    OLED_ShowNum_1206(36,45,setting_amount[mark_setting_amount]+(float)10,1);
  
//  OLED_ShowString_1206(0,39,"PreDirErr=",1);
//  char num_arr[3];
//  num_arr[0] = cPreDirErr / 100+48;
//  num_arr[1] = cPreDirErr / 10 % 10+48;
//  num_arr[2] = cPreDirErr % 10+48;
//  OLED_ShowString_1206(66,39,num_arr,1);
  
  // 刷新显存，显示图像
  OLED_Refresh_Gram();
}

// 刹车
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

// int绝对值输出
int AbsoluteValue(int v){
  if(v<0)v=-v;
  return v;
}