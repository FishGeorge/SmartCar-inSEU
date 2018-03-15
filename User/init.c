/*********************************************************
  @2017CameraDemo
  @固件库：超核V2.4
  @author：wgq & lq
  @2017.11.27
  @for seu2017 摄像头组
*********************************************************/

#include "init.h"
#include "varieble.h"
#include "isr.h"
#include "oled.h"
#include "sysinit.h"
#include "DEV_SCCB.h"

void init_ov7620(void){
  
    //场中断
    GPIO_QuickInit(HW_GPIOC, 6, kGPIO_Mode_IPD);
    GPIO_CallbackInstall(HW_GPIOC, GPIO_ISR);
    GPIO_ITDMAConfig(HW_GPIOC, 6, kGPIO_IT_RisingEdge, false);//上升沿沿触发
    //行中断
    GPIO_QuickInit(HW_GPIOC, 7, kGPIO_Mode_IPU);
    GPIO_CallbackInstall(HW_GPIOC, GPIO_ISR);
    GPIO_ITDMAConfig(HW_GPIOC, 7, kGPIO_IT_RisingEdge,false);//上升沿触发
    //PCLK
    GPIO_QuickInit(HW_GPIOC, 2, kGPIO_Mode_IPU);
    GPIO_ITDMAConfig(HW_GPIOC, 2, kGPIO_DMA_RisingEdge,false);
    
    //摄像头数据口，下拉
    GPIO_QuickInit(HW_GPIOC, 8, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 9, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 10, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 11, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 12, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 13, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 14, kGPIO_Mode_IPD);
    GPIO_QuickInit(HW_GPIOC, 15, kGPIO_Mode_IPD);    
    
    GPIO_ITDMAConfig(HW_GPIOC, 6, kGPIO_IT_RisingEdge, true);
    GPIO_ITDMAConfig(HW_GPIOC, 7, kGPIO_IT_RisingEdge,true);
    GPIO_ITDMAConfig(HW_GPIOC, 2, kGPIO_DMA_RisingEdge,true); 
    
    //DMA初始化
    DMA_InitTypeDef DMA_InitStruct1 = {0};
    DMA_InitStruct1.chl = HW_DMA_CH0;                                           
    DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ;                             
    DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;             
    DMA_InitStruct1.minorLoopByteCnt = 1;
    DMA_InitStruct1.majorLoopCnt = col_num;//主循环采集数，即一次主循环采集一行

    DMA_InitStruct1.sAddr = (uint32_t)&(PTC->PDIR)+1;//dma源地址：ptc8~15                           
    DMA_InitStruct1.sLastAddrAdj = 0;                                          
    DMA_InitStruct1.sAddrOffset = 0;                                           
    DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;//数据宽度                           
    DMA_InitStruct1.sMod = kDMA_ModuloDisable;                                 

    //DMA_InitStruct1.dAddr = (uint32_t)DestBuffer;
    DMA_InitStruct1.dLastAddrAdj = 0;
    DMA_InitStruct1.dAddrOffset = 1;
    DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
    DMA_InitStruct1.dMod = kDMA_ModuloDisable;
    
    DMA_Init(&DMA_InitStruct1);
    DMA_DisableRequest(HW_DMA_CH0);//先关闭DMA传输
    
    //配置摄像头寄存器
    uint8_t i=0;
    //初始化SCCB所需引脚，SCCB是一种简化的类似于I2C的通信协议。
    //初始化PTC3和PTC0，作为SCCB中的SDA和SCL。
    GPIO_QuickInit(HW_GPIOC, 0, kGPIO_Mode_OPP);
    GPIO_QuickInit(HW_GPIOC, 3, kGPIO_Mode_OPP);
    while(i==0)
      i += LPLD_SCCB_WriteReg(0x14,0x24);		//QVGA(320*120)
    while(i==1)
      i += LPLD_SCCB_WriteReg(0x24, 0x20);		//连续采集模式(320*240)
    while(i==2)
      i += LPLD_SCCB_WriteReg(0x70, 0xc1);		//驱动电流增加一倍
    while(i==3)
      i += LPLD_SCCB_WriteReg(0x06, 0xa0);		//亮度控制
}
