
int main(){
  DelayInit();
  UART_QuickInit(UART3_RX_PC16_TX_PC17,115200);
  GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);//LED7
  //uint16_t data=0;
  PEout(25)=0;
  while(1){
//    if(!UART_ReadByte(HW_UART3,&data)){
//      PEout(25)=1;
//      UART_WriteByte(HW_UART3,data);
//      DWT_DelayMs(500);
//      PEout(25)=0;
//    }
    UART_WriteByte(HW_UART3,'g');
  }
}