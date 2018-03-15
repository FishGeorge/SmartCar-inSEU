#include "chlib_k.h"
/*
**** 通过单片机设置HC05 By Gong ****
* 接线方式 TX-RX,RX-TX
* 将这段代码粘贴替换demo的main()
* 请填写下方编译不通过处的蓝牙名称设定
* 烧写程序后完全断电，按住HC05蓝牙串口的key上电
* 大概一秒后松手，HC05状态灯 慢单闪（一秒一亮）为正常（进入AT模式）
* 设置操作流程：
*  1）demo板灯LED8亮时，按demo板KEY1
*  2）此时灯LED8应灭，随后LED7亮，0.5s后灯LED7灭，表示此环节的设置通过
*  3）待LED8再次点亮，重复操作1）2）
*  4）灯LED7 LED8同时点亮，表明设置完毕，断电，烧写其他的程序就可以用了
*
* 初始波特率：38400Hz（开机AT模式）
* AT                      进入AT模式（还是要发这个）
* AT+NAME="XXX"           修改蓝牙模块名称为XXX
* AT+ROLE=0               蓝牙模式为从模式
* AT+CMODE=1              蓝牙连接模式为任意地址连接模式，也就是说该模块可以被任意蓝牙设备连接
* AT+PSWD=1234            蓝牙配对密码为1234
* AT+UART=115200,0,0        蓝牙通信串口波特率为115200，停止位1位，无校验位
* AT+RMAAD                清空配对列表
*
**/
int main(){
  DelayInit();
  UART_QuickInit(UART3_RX_PC16_TX_PC17,38400);
  GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_OPP);//LED7
  GPIO_QuickInit(HW_GPIOC, 18, kGPIO_Mode_OPP);//LED8
  GPIO_QuickInit(HW_GPIOB, 11, kGPIO_Mode_IPD);//KEY1
  uint16_t data;
  PCout(18)=0;
  
  while(1){
    PCout(18)=1;
    if(PCout(18)==1&&PBin(11)==1){
      //DWT_DelayMs(500);
      PCout(18)=0;
      PEout(25)=1;
      DWT_DelayMs(500);
      PEout(25)=0;
      break;
    }
  }
  UART_printf(HW_UART3,"AT\r\n");
  while(1){
    if(!UART_ReadByte(HW_UART3,&data)){
      PCout(18)=1;
    }
    if(PCout(18)==1&&PBin(11)==1){
      //DWT_DelayMs(500);
      PCout(18)=0;
      PEout(25)=1;
      DWT_DelayMs(500);
      PEout(25)=0;
      break;
    }
  }
 UART_printf(HW_UART3,"AT+NAME=RedBlooth\r\n");//此处缺一分号编译通过不了，以提醒用户修改蓝牙名
  while(1){
    if(!UART_ReadByte(HW_UART3,&data)){
      PCout(18)=1;
    }
    if(PCout(18)==1&&PBin(11)==1){
      //DWT_DelayMs(500);
      PCout(18)=0;
      PEout(25)=1;
      DWT_DelayMs(500);
      PEout(25)=0;
      break;
    }
  }
  UART_printf(HW_UART3,"AT+ROLE=0\r\n");
  while(1){
    if(!UART_ReadByte(HW_UART3,&data)){
      PCout(18)=1;
    }
    if(PCout(18)==1&&PBin(11)==1){
      //DWT_DelayMs(500);
      PCout(18)=0;
      PEout(25)=1;
      DWT_DelayMs(500);
      PEout(25)=0;
      break;
    }
  }
  UART_printf(HW_UART3,"AT+PSWD=1234\r\n");
  while(1){
    if(!UART_ReadByte(HW_UART3,&data)){
      PCout(18)=1;
    }
    if(PCout(18)==1&&PBin(11)==1){
      //DWT_DelayMs(500);
      PCout(18)=0;
      PEout(25)=1;
      DWT_DelayMs(500);
      PEout(25)=0;
      break;
    }
  }
  UART_printf(HW_UART3,"AT+UART=115200,0,0\r\n");
  while(1){
    if(!UART_ReadByte(HW_UART3,&data)){
      PCout(18)=1;
    }
    if(PCout(18)==1&&PBin(11)==1){
      //DWT_DelayMs(500);
      PCout(18)=0;
      PEout(25)=1;
      DWT_DelayMs(500);
      PEout(25)=0;
      break;
    }
  }
  UART_printf(HW_UART3,"AT+RMAAD\r\n");
  while(1){
    if(!UART_ReadByte(HW_UART3,&data)){
      PCout(18)=1;
      DWT_DelayMs(1000);
      PEout(25)=1;
      break;
    }
  }
  
  while(1){
      PEout(25)=1;
      PCout(18)=1;
  }
}