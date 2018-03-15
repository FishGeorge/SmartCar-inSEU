
int main(){ 
//  DelayInit();
  SYSTICK_DelayInit();
  
  FTM_PWM_QuickInit(FTM0_CH3_PC04, kPWM_EdgeAligned, 100, 1000);
  
  while(1){
    for(int i=0;i<100;i++){
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH3,100*i);
      SYSTICK_DelayMs(20);
    }
    SYSTICK_DelayMs(1000);
    for(int i=0;i<100;i++){
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH3,10000-100*i);
      SYSTICK_DelayMs(20);
    }
    SYSTICK_DelayMs(500);
  }
}