void IsrTest(void){
    PCout(18)=1;
    PIT_ITDMAConfig(HW_PIT_CH1,kPIT_IT_TOF,ENABLE);
  }

void IsrTest2(void){
    PCout(18)=0;
    PIT_ITDMAConfig(HW_PIT_CH1,kPIT_IT_TOF,DISABLE);
    //PIT_ITDMAConfig(HW_PIT_CH2,kPIT_IT_TOF,ENABLE);
}

void IsrTest3(void){
    PCout(18)=1;
    PIT_ITDMAConfig(HW_PIT_CH2,kPIT_IT_TOF,DISABLE);
    PIT_ITDMAConfig(HW_PIT_CH3,kPIT_IT_TOF,ENABLE);
}

void IsrTest4(void){
    PCout(18)=0;
    PIT_ITDMAConfig(HW_PIT_CH3,kPIT_IT_TOF,DISABLE);
}

int main(){
  GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_OPP);
  SYSTICK_DelayInit();
  
  PIT_QuickInit(HW_PIT_CH0,3000000);
  PIT_ITDMAConfig(HW_PIT_CH0,kPIT_IT_TOF,ENABLE);
  PIT_CallbackInstall(HW_PIT_CH0,IsrTest);
  
  PIT_QuickInit(HW_PIT_CH1,100000);
  PIT_ITDMAConfig(HW_PIT_CH1,kPIT_IT_TOF,DISABLE);
  PIT_CallbackInstall(HW_PIT_CH1,IsrTest2);
  
  PIT_QuickInit(HW_PIT_CH2,100000);
  PIT_ITDMAConfig(HW_PIT_CH2,kPIT_IT_TOF,DISABLE);
  PIT_CallbackInstall(HW_PIT_CH2,IsrTest3);
  
  PIT_QuickInit(HW_PIT_CH3,100000);
  PIT_ITDMAConfig(HW_PIT_CH3,kPIT_IT_TOF,DISABLE);
  PIT_CallbackInstall(HW_PIT_CH3,IsrTest4);
  
  
  while(1){
//    SYSTICK_DelayMs(500);
//    PCout(18)=0;
//    SYSTICK_DelayMs(500);
//    PCout(18)=1;
  }
}
