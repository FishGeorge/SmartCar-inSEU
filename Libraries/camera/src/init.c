/*********************************************************/
//@demo
//@�̼��⣺����V2.4
//@author��th
//@2016.11.30
//@for seu2016 ����ͷ������
/*********************************************************/

#include "init.h"
#include "varieble.h"
#include "isr.h"
#include "sysinit.h"
#include "DEV_SCCB.h"
#include "oled.h"
//��ʼ��
void init(void){

    init_steer();//�����ʼ��

}

void init_ov7620(void){
        //DMA��ʼ��
        DMA_InitTypeDef DMA_InitStruct1 = {0};//�����ʼ���ṹ��
        DMA_InitStruct1.chl = HW_DMA_CH0;                                           
        DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ;                             
        DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;             
        DMA_InitStruct1.minorLoopByteCnt = 1;
        DMA_InitStruct1.majorLoopCnt = col_num;//��ѭ���ɼ�������һ����ѭ���ɼ�һ��
    
        DMA_InitStruct1.sAddr = (uint32_t)&(PTC->PDIR)+1;//dmaԴ��ַ��ptc8~15                           
        DMA_InitStruct1.sLastAddrAdj = 0;                                          
        DMA_InitStruct1.sAddrOffset = 0;                                           
        DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;//���ݿ��                           
        DMA_InitStruct1.sMod = kDMA_ModuloDisable;                                 
    
        DMA_InitStruct1.dLastAddrAdj = 0;
        DMA_InitStruct1.dAddrOffset = 1;
        DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
        DMA_InitStruct1.dMod = kDMA_ModuloDisable;
        
        DMA_Init(&DMA_InitStruct1);
        DMA_DisableRequest(HW_DMA_CH0);//�ȹر�DMA����
        
        //��������ͷ�Ĵ���
        uint8_t i=0;
        //��ʼ��SCCB�������ţ�SCCB��һ�ּ򻯵�������I2C��ͨ��Э�顣
        //��ʼ��PTC3��PTC0����ΪSCCB�е�SDA��SCL��
        GPIO_QuickInit(HW_GPIOC, 0, kGPIO_Mode_OPP);
        GPIO_QuickInit(HW_GPIOC, 3, kGPIO_Mode_OPP);
        while(i==0)
          i += LPLD_SCCB_WriteReg(0x14,0x24);		//QVGA(320*120)
        while(i==1)
          i += LPLD_SCCB_WriteReg(0x70, 0xc1);		//������������һ��
        while(i==2)
          i += LPLD_SCCB_WriteReg(0x24, 0x20);		//�����ɼ�ģʽ(320*240)
        while(i==3)
          i += LPLD_SCCB_WriteReg(0x06, 0xa0);		//���ȿ���

              

}


void init_steer(void){
  //ռ�ձ� = pwmDuty/10000*100%
  //������ֵռ�ձ�=1.5/20=7.5%,ʵ������ݻ�е�ṹ�Ͷ��������е���
  FTM_PWM_QuickInit(FTM2_CH0_PB18,kPWM_EdgeAligned,50,0);
  //FTM_PWM_InvertPolarity(HW_FTM2,HW_FTM_CH0,kFTM_PWM_LowTrue);
  
}