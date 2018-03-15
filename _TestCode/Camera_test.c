
int main(){
  OLED_Init();
  DisableInterrupts;
  init_ov7620();
  EnableInterrupts;
  
  //GPIO_QuickInit(HW_GPIOE,25,kGPIO_Mode_OPP);
  //PEout(25)=1;
  UART_QuickInit(UART3_RX_PC16_TX_PC17,115200);
  uint32_t i,j;
  while(1){
    searchline_OV7620();
    UART_WriteByte(HW_UART3,0X01);
    UART_WriteByte(HW_UART3,0XFE);
    
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
}