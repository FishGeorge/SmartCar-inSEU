
int main(){
  //����PE25Ϊ..���
  GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);
  
  GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IPD);
  //����PE25����ߵ�ƽ
//  PEout(25)=1;
  //keep K60 working
  while(1){
    if(PBin(21)==0)
      PEout(25)=1;
    else
      PEout(25)=0;
  }
}