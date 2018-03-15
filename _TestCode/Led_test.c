
int main(){
  //设置PE25为..输出
  GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
  
  GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IPD);
  //设置PE25输出高电平
//  PEout(25)=1;
  //keep K60 working
  while(1){
    if(PBin(21)==0)
      PEout(25)=1;
    else
      PEout(25)=0;
  }
}