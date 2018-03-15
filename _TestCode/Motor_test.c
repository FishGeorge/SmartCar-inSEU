int main(){
  FTM_PWM_QuickInit(FTM0_CH5_PD05,kPWM_EdgeAligned,10000,500);
  FTM_PWM_QuickInit(FTM0_CH7_PD07,kPWM_EdgeAligned,10000,0);
  while(1){
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,1000);
  }
}