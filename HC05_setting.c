#include "chlib_k.h"
/*
**** ͨ����Ƭ������HC05 By Gong ****
* ���߷�ʽ TX-RX,RX-TX
* ����δ���ճ���滻demo��main()
* ����д�·����벻ͨ���������������趨
* ��д�������ȫ�ϵ磬��סHC05�������ڵ�key�ϵ�
* ���һ������֣�HC05״̬�� ��������һ��һ����Ϊ����������ATģʽ��
* ���ò������̣�
*  1��demo���LED8��ʱ����demo��KEY1
*  2����ʱ��LED8Ӧ�����LED7����0.5s���LED7�𣬱�ʾ�˻��ڵ�����ͨ��
*  3����LED8�ٴε������ظ�����1��2��
*  4����LED7 LED8ͬʱ����������������ϣ��ϵ磬��д�����ĳ���Ϳ�������
*
* ��ʼ�����ʣ�38400Hz������ATģʽ��
* AT                      ����ATģʽ������Ҫ�������
* AT+NAME="XXX"           �޸�����ģ������ΪXXX
* AT+ROLE=0               ����ģʽΪ��ģʽ
* AT+CMODE=1              ��������ģʽΪ�����ַ����ģʽ��Ҳ����˵��ģ����Ա����������豸����
* AT+PSWD=1234            �����������Ϊ1234
* AT+UART=115200,0,0        ����ͨ�Ŵ��ڲ�����Ϊ115200��ֹͣλ1λ����У��λ
* AT+RMAAD                �������б�
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
 UART_printf(HW_UART3,"AT+NAME=RedBlooth\r\n");//�˴�ȱһ�ֺű���ͨ�����ˣ��������û��޸�������
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