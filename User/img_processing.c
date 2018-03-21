/*********************************************************
@2017CameraDemo
@固件库：超核V2.4
@author：wgq & lq
@2017.11.27
@for seu2017 摄像头组
*********************************************************/

#include "varieble.h"
#include "img_processing.h"
#include "oled.h"

//char* out;
int whiteRoad=0;
/*
* @name		searchline_OV7620
* @description	To get the racing track from the imgadd.
* @inputval	None
* @outputval	None
* @retval 	0
*/
void searchline_OV7620(int wb)
{
  whiteRoad=wb;
  int16_t lp1, lp2; //第0行时，lp1,lp2从上一次的线的第零行开始扫描
  int16_t lp3, lp4;
  uint8_t CurL = 0; //指向当前的处理的行
  uint8_t LastL = 0;            //上一行
  uint8_t LastR = 0;
  uint8_t isLeftEdge = 0;       //是否看到左右边界
  uint8_t isRightEdge = 0;      //是否看到左右边界
  uint8_t *p;
  uint8_t BlackcntL = 0;      //记录出现几个黑点
  uint8_t BlackcntR = 0;
  uint8_t LRightEdge, LLeftEdge;
  uint8_t i = 0, j = 0;
  uint8_t xx = 0;
  
  if (Lx[L_Start[0]] != col_num && Lx[L_Start[0]] != 0 && L_Start[0]<15)     //前十五行有非空左线起点
  {
    lp1 = (Lx[L_Start[0]]>1 + offset[L_Start[0]]) ? Lx[L_Start[0]] - offset[L_Start[0]] : 1;
    lp2 = lp1 + P_WIDTH;
  }
  else  //前一半都没有找到参考值
  {
    if (Rx[R_Start[0]] != 0 && R_Start[0]<15)
    {
      lp1 = ((int16_t)((int16_t)Rx[R_Start[0]] + (int16_t)offset[R_Start[0]])>col_num - P_WIDTH - 10) ? col_num - P_WIDTH - 10 : Rx[R_Start[0]] + offset[R_Start[0]];
      lp2 = lp1 + P_WIDTH;
    }
    else
    {
      lp1 = car_center;
      lp2 = lp1 + P_WIDTH;
    }
  }
  if (Rx[R_Start[0]] != 0 && R_Start[0]<15)
  {
    lp3 = ((int16_t)((int16_t)Rx[R_Start[0]] + (int16_t)offset[R_Start[0]])<col_num - P_WIDTH - 5) ? Rx[R_Start[0]] + offset[R_Start[0]] : col_num - P_WIDTH - 5;
    lp4 = lp3 + P_WIDTH;
  }
  else  //如果前面一直没有找到边，那么就依照找到的左线的位置推测lp1,lp2位置
  {
    if (Lx[L_Start[0]] != col_num&&L_Start[0]<15)
    {
      lp3 = (Lx[L_Start[0]]>5 + offset[L_Start[0]]) ? Lx[L_Start[0]] - offset[L_Start[0]] : 5;
      lp4 = lp3 + P_WIDTH;
    }
    else
    {
      lp3 = car_center;
      lp4 = lp3 + P_WIDTH;
    }
  }
  for (CurL = 0; CurL<row_num; CurL++){
    // 指向当前行
    p = imgadd + CurL*col_num;
    if (CurL>0){                                                                        //确定左右线扫描起点  
      LastL = CurL - 1;
      // Lx[]=col_num表示没有找到左黑线
      // 在本行前五行找到上一个线中心
      while (LastL>0 && CurL - LastL<5 && Lx[LastL] == col_num)
        LastL--;
      if (Lx[LastL] != col_num)
      {
        if (Lx[LastL]>offset[LastL])
        {
          if (Rx[CurL - 1] != 0 && Lx[LastL] - offset[LastL]<Rx[CurL - 1] + 5)            //如果扫线开始点在上一行右线的右边 则以上一行的右线作为扫线开始
            lp1 = Rx[CurL - 1] + 5;
          else
          {
            lp1 = Lx[LastL] - offset[LastL];                                        //需修改 ？？？？？？？？
          }
        }
        else
        {
          if (Rx[CurL - 1] != 0 && Rx[CurL - 1] + 5<col_num - P_WIDTH)
            lp1 = Rx[CurL - 1] + 5;
          else
            lp1 = 1;
        }
        lp2 = lp1 + P_WIDTH;
      }
      // 前面一直无可搜寻到的左线
      else
      {
        if (Rx[CurL - 1] != 0)                  //那么如果第零行是用右线确定的，则用右线的推出来值
        {
          lp1 = ((int16_t)((int16_t)Rx[CurL - 1] + 5)>col_num - P_WIDTH - 10) ? col_num - P_WIDTH - 10 : Rx[CurL - 1] + 5;
          lp2 = lp1 + P_WIDTH;
        }
        else
        {
          lp1 = car_center;
          lp2 = lp1 + P_WIDTH;
        }
      }
      //确定方向右线扫描起点
      LastR = CurL - 1;
      while (LastR>0 && CurL - LastR<5 && Rx[LastR] == 0)LastR--;     //LastL中记录第一个找到的有效左黑线中心所在行数
      if (Rx[LastR] != 0)                                             //找到上一个右线中心
      {
        if ((int16_t)((int16_t)Rx[LastR] + (int16_t)offset[LastR])<col_num - P_WIDTH - 5)
        {
          if (Lx[CurL - 1] != col_num && (int16_t)(Rx[LastR] + offset[LastR])>(int16_t)(Lx[CurL - 1] - P_WIDTH - 5))              //如果扫线开始点在上一行左线的左边 则以上一行的左线作为扫线开始
            lp3 = Lx[CurL - 1] - P_WIDTH - 5;
          else
          {
            lp3 = Rx[LastR] + offset[LastR];                                        //需修改
          }
        }
        else
        {
          if (Lx[CurL - 1] == col_num){
            lp3 = col_num - P_WIDTH - 5;
          }
          else if (Lx[CurL - 1]>5 + P_WIDTH)lp3 = Lx[CurL - 1] - P_WIDTH - 5;
          else lp3 = car_center;
        }
        lp4 = lp3 + P_WIDTH;
      }
      else //之前一直没有找到右线
      {
        if (Lx[CurL - 1] != col_num)//按照本行左线的位置 确定lp1 lp2
        {
          lp3 = ((int16_t)(Lx[CurL - 1]>5 + P_WIDTH + 5)) ? Lx[CurL - 1] - P_WIDTH - 5 : 5;
          lp4 = lp3 + P_WIDTH;
        }
        else{ //本行左线没有
          lp3 = car_center;
          lp4 = lp3 + P_WIDTH;
        }
      }
      //左右线扫线开始
    }
    if (lp3>0) //找左右边界
    {
      //------找线右边界-------------------
      while (lp3>0 && !isRightEdge)
      {
        if ((int16_t)(*(p + lp4))>BW_DELTA + (int16_t)*(p + lp3))		//利用有符号的来消除噪点
        {
          while ((int16_t)(*(p + lp4))>BW_DELTA + (int16_t)*(p + lp3) && lp3>0)
          {
            if ((int16_t)(*(p + lp4))<255)
              BlackcntR++;
            lp3--;
            lp4--;
            if (BlackcntR >= LINE_EDGE)break;
          }
          if (BlackcntR >= LINE_EDGE) //判断找到黑线
          {
            LRightEdge = lp3 + LINE_EDGE;
            BlackcntR = 0;
            
            xx = 0;
            for (i = 0; i<10; i++){
              if (*(p + lp3 + LINE_EDGE + i)>whiteRoad)xx++;
            }
            if (xx>6)
              isRightEdge = 1;
            else
              BlackcntR = 0;
          }
          else{
            BlackcntR = 0;
          }
        }
        else
        {
          lp3--;
          lp4--;
        }
      }
    }
    if (lp2<col_num)                    //找左右边界,从内向外扫描
    {
      //------找线左边界-------------------
      while (lp2<col_num && !isLeftEdge)		//当lp2没有到达列的最大值继续扫描，右扫描模式
      {
        if (((int16_t)(*(p + lp1)))>BW_DELTA + (int16_t)*(p + lp2))			// 
        {
          while ((int16_t)(*(p + lp1))>BW_DELTA + (int16_t)*(p + lp2) && lp2<col_num)
          {
            if ((int16_t)(*(p + lp1))<255)BlackcntL++;
            lp1++;
            lp2++;
            if (BlackcntL >= LINE_EDGE)break;
          }
          if (BlackcntL >= LINE_EDGE)			//找到左边界退出循环，lp1和lp2间隔设为1,	                    
          {
            LLeftEdge = lp2 - LINE_EDGE;
            BlackcntL = 0;
            
            xx = 0;
            for (j = 0; j<10; j++){
              if (*(p + lp2 - LINE_EDGE - j)>whiteRoad)xx++;
            }
            if (xx>6)
              isLeftEdge = 1;
            else
              BlackcntL = 0;
          }
          else					//遇到噪点，计数清零
            BlackcntL = 0;
        }
        else
        {
          lp1++;
          lp2++;
        }
      }
    }
    if (isLeftEdge){
      isLeftEdge = 0;
      Lx[CurL] = LLeftEdge;
    }
    else Lx[CurL] = col_num;
    if (isRightEdge){
      isRightEdge = 0;
      Rx[CurL] = LRightEdge;
    }
    else Rx[CurL] = 0;
    
    Cx[CurL]=(Lx[CurL]+Rx[CurL])/2;
    
  }
}
/*
* @name		dispimage
* @description	Display the image or racing track on OLED screen.
* @inputval	None
* @outputval	None
* @retval 	0
*/
unsigned char display_col[158]={
  0,0,1,2,3,4,4,5,6,7,8,8,9,10,11,12,12,13,14,
  15,16,17,17,18,19,20,21,21,22,23,24,25,25,26,27,
  28,29,29,30,31,32,33,34,34,35,36,37,38,38,39,40,
  41,42,42,43,44,45,46,46,47,48,49,50,51,51,52,53,
  54,55,55,56,57,58,59,59,60,61,62,63,64,64,65,66,
  67,68,68,69,70,71,72,72,73,74,75,76,76,77,78,79,
  80,81,81,82,83,84,85,85,86,87,88,89,89,90,91,92,
  93,93,94,95,96,97,98,98,99,100,101,102,102,103,104,
  105,106,106,107,108,109,110,110,111,112,113,114,115,
  115,116,117,118,119,119,120,121,122,123,123,124,125,126,127};
// oled显示自定义（放缩，将158宽的图像压缩到128宽）

#define OLED_Black 0
#define OLED_White 1

void dispimage(int n,int cDirErr,int cPreDirErr,int cSpeed,int MotorFrequency){
  // 清除上一周期OLED显示图像（可能导致频闪）
  if(!(n%3))// 每3个周期clear一次，降低频闪
    OLED_Clear();
  
  // 赛道二元化图像显示
  // OLED像素128x64，只用了63-14行（倒置使用）
  for(int j=0;j<row_num;j++){
    // 左右边界显示，并涂白外界
    OLED_Fill(0,63-j,127-display_col[Lx[j]],63-j,OLED_White);
    OLED_Fill(127-display_col[Rx[j]],63-j,127,63-j,OLED_White);
//    // 左右边界显示，不涂白外界
//    OLED_DrawPoint(127-display_col[Lx[j]],63-j,OLED_White);
//    OLED_DrawPoint(127-display_col[Rx[j]],63-j,OLED_White);
    // 中线显示.1
//  OLED_DrawPoint((uint8_t)(127-display_col[Cx[j]]),63-j,OLED_White);
    // 中线显示.2
    OLED_DrawPoint((uint8_t)(127-(display_col[Lx[j]]+display_col[Rx[j]])/2),63-j,OLED_White);
  }
  // 第13行作直线
  OLED_Fill( 0,13,127,13,OLED_White);
  
  // 车况数据显示
  char num_arr1[4];
  if(cDirErr<0)cDirErr=-cDirErr;
  num_arr1[0] = cDirErr / 10 % 10+48;
  num_arr1[1] = cDirErr % 10+48;
  num_arr1[2] = 0;
  num_arr1[3] = 0;
  OLED_ShowString_1206(0,0,"E=",1);
  OLED_ShowString_1206(12,0,num_arr1,1);
  
  if(cPreDirErr<0)cPreDirErr=-cPreDirErr;
  num_arr1[0] = cPreDirErr / 10 % 10+48;
  num_arr1[1] = cPreDirErr % 10+48;
  num_arr1[2] = 0;
  num_arr1[3] = 0;
  OLED_ShowString_1206(26,0,"PE=",1);
  OLED_ShowString_1206(44,0,num_arr1,1);
  
  num_arr1[0] = cSpeed / 100+48;
  num_arr1[1] = cSpeed / 10 % 10+48;
  num_arr1[2] = cSpeed % 10+48;
  num_arr1[3] = 0;
  OLED_ShowString_1206(58,0,"S=",1);
  OLED_ShowString_1206(70,0,num_arr1,1);
  
  num_arr1[0] = MotorFrequency / 1000+48;
  num_arr1[1] = MotorFrequency / 100 % 10+48;
  num_arr1[2] = MotorFrequency / 10 % 10+48;
  num_arr1[3] = MotorFrequency % 10+48;
  OLED_ShowString_1206(90,0,"F=",1);
  OLED_ShowString_1206(102,0,num_arr1,1);
  
  // 刷新显存，显示图像
  OLED_Refresh_Gram();
}
//
//int num_count(int num){
//  int out=0;
//  while(num!=0){
//    num=num/10;
//    out++;
//  }
//  return out;
//}
//
//char* inttochar(int num){
//  int count=num_count(num);
//  char o[count];
//  for(int i=count;i>0;i--){
//    out[i-1]=num/10+48;
//  }
//  return out;
//}