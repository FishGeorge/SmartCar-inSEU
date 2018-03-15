int main(){
  FTM_PWM_QuickInit(FTM2_CH0_PB18,kPWM_EdgeAligned,50,500);
//  UART_QuickInit(UART3_RX_PC16_TX_PC17,115200);
  while(1){
    FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,730);
//    for(int i=50;i<100;i++)
//      FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,10*i);
//    for(int i=100;i>50;i--)
//      FTM_PWM_ChangeDuty(HW_FTM2,HW_FTM_CH0,10*i);
  }
}