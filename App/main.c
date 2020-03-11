/*
超低速：1750
低速  ：1800
中速  ：2000
高速  ：2200
差速  ：1950    1300






1.启动扫描行数
2.找灯的扫描行数
3.反光是否存在
4.限制判断为灯的条件，必须修改
5.duoji 大修改



*/
#include "common.h"
#include "include.h"
#include "math.h"
#include "MK60_FTM.h"

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH2
#define MOTOR4_PWM  FTM_CH5

#define MOTOR1_PWM_IO  FTM0_CH3
#define MOTOR2_PWM_IO  FTM0_CH4
#define MOTOR3_PWM_IO  FTM0_CH2
#define MOTOR4_PWM_IO  FTM0_CH5

#define MOTOR_HZ    (18*1000)
//1470        1620     1340
#define dj_mid          1510                         //舵机居中  10000-1550
#define dj_left_max       1640                           //左极限    10000-1675
#define dj_right_max      1380
//1640         1380       1510
#define  range      40                                              //光电开关的极限距离
#define meideng     40                                             //灭灯极限距离

int16 duty;                                            //舵机占空比      
uint8 duoji_error;
int16 duty_last;
int16 duty_tly_last=0;
uint8 dengflag=0;
uint8 deng_tly_last=0;
uint8 deng_i_last=0,deng_j_last=0;
uint8 dengflag1,dengflag_last=0;
uint8 bdflag;
uint8 deng_i=0,deng_j=0;
int8 zhuanwan=-5;
uint8 n=0;
uint32 num=0;
uint8 raodengflag;
uint8  imgbuff[CAMERA_SIZE];                                 //定义存储接收图像的数组
uint8  img[CAMERA_H][CAMERA_W];                              //二维图像解压数组
int32 sudu;
int8 zhangai;
uint8 leixin;
uint8 piaoyi_right_flag;
uint8 piaoyi_left_flag;
uint8 piaoyi_flag;
uint8 bm1,bm2,bm3,bm4;
uint16 py_num;
uint8 gdflag;                      //过灯
uint8 jbzflag;                     //近处避障
uint8 ybzflag;                     //远处避障
uint8 zqflag;                   //转圈
uint8 yxdflag;                    //远处寻灯
uint8 jxdflag;                    //近处寻灯

int16 leftsudu;
int16 rightsudu;

int16 mid[60]={10,40,40,40,40,40,40,40,40,36,
               36,36,36,36,35,37,35,34,33,32,//15/37   17/34   19/                      
               31,29,28,26,25,24,23,23,22,21,//20/31   22/28   25/24   28/22
               21,21,21,20,19,18,17,16,15,14,//32/21     34/19   39/14    
               11,11,11,11,11,11,11,11,11,11,                    //42/11
               11,11,11,11,11,11,11,11,11,11};//1500
/*int16 mid[60]={10,40,40,40,40,40,40,40,40,40,
               40,40,40,40,40,39,38,37,36,36,
               35,34,33,32,31,30,29,28,27,26,
               25,24,23,22,21,20,19,18,17,16,
               16,15,14,1,13,13,12,11,10,9,
               25,25,25,25,25,25,25,25,25,25};*/
uint8 Re_buf[11],counter;
uint8 ucStra[6],ucStrw[6],ucStrAngle[6];
double ZOUT;
double ZOUT_LAST;
double ZOUT_LAST2;
double ZOUT_LAST3;
uint16 gdkg_error_num=0;                  //光电开关电机堵转计数
uint8 gdkg_error_flag;                   //光电开关电机堵转标志位
uint8 dj_left_stop;
uint8 dj_right_stop;
uint8 tly_num;

uint8 qd_flag;
uint8 qd_num;
uint8 tly_flag;

void PORTA_IRQHandler();
void PIT0_IRQHandler2();
void DMA0_IRQHandler();
void duoji();                                              //舵机
void lcd_camera_init();                                    //lcd和摄像头初始化并配置中断函数
void lcd_display();                                        //lcd显示图像
void flaglcd_display();
void duochongpanduan();                       //尝试添加多重判断
void zhaodeng();
void gdkg();
void raodeng();
void twenty();
void dianji_init();
void dianji_chaodisu();
void dianji_chasu();
void dianji_disu();
void dianji_zhongsu();
void dianji_gaosu();
void dianji_fanzhuan();
void dianji_buzhuan();
void uart2_handler(void);
//void dianji_zhongsu_youzhuan();
//void dianji_zhongsu_zuozhuan();
void tly();
void qidong();
void tly_duoji();
void duoji_xuanze();



/*******************************舵机*******************************************/
void duoji()
{ 
gdflag=0;
jbzflag=0;
ybzflag=0;
zqflag=0;
yxdflag=0;
jxdflag=0;

  if(dengflag==0)
  {
   // if(zhangai==0)
    //{
       duty=dj_left_max;
       ftm_pwm_duty(FTM3 ,FTM_CH6,duty);
       zqflag=1;
       gdflag=0;
       jbzflag=0;
       ybzflag=0;
       yxdflag=0;
       jxdflag=0;
   // }
   // else if(zhang!=0)
    //  ;
  }
  else if(dengflag==1 && deng_i<=38)
  {
    if(deng_i<24)   /*xiugai   确定deng_i*/                    //
    {
      gdkg();
      if(zhangai==0)
      {
        yxdflag=1;
        gdflag=0;
        jbzflag=0;
        ybzflag=0;
        zqflag=0;
        jxdflag=0;
        duty=dj_mid-3*(deng_j-mid[deng_i]);/*xiugai   确定P*/
        if(duty>dj_left_max)
          duty=dj_left_max;
        if(duty<dj_right_max)
          duty=dj_right_max;
        ftm_pwm_duty(FTM3 ,FTM_CH6,duty);
      }
      else if(zhangai!=0)
      {
        if(deng_i<20)
        {
          ybzflag=1;
          gdflag=0;
          jbzflag=0;
          zqflag=0;
          yxdflag=0;
          jxdflag=0;
          duty=dj_mid-50*zhangai;
        }
        else if(deng_i>=20&&deng_i<24)
        {
          jbzflag=1;
          gdflag=0;
          ybzflag=0;
          zqflag=0;
          yxdflag=0;
          jxdflag=0;
        duty=dj_mid-40*zhangai;
        }
        if(duty>dj_left_max)
          duty=dj_left_max;
        if(duty<dj_right_max)
          duty=dj_right_max;
        ftm_pwm_duty(FTM3 ,FTM_CH6,duty);
        
        DELAY_MS(90);
      }
    }
    
    else if(deng_i>=24&&deng_i<=38)
    {
      jxdflag=1;
      gdflag=0;
      jbzflag=0;
      ybzflag=0;
      zqflag=0;
      yxdflag=0;

      duty=dj_mid-3.5*(deng_j-mid[deng_i]);
        if(duty>dj_left_max)
          duty=dj_left_max;
        if(duty<dj_right_max)
          duty=dj_right_max;
        ftm_pwm_duty(FTM3 ,FTM_CH6,duty);
    }
  }
  else if(dengflag==1&&deng_i>38)
  {
    
      /*duty=dj_left_max;
       ftm_pwm_duty(FTM3 ,FTM_CH6,duty);*/
        gdflag=1;
        jbzflag=0;
        ybzflag=0;
        zqflag=0;
        yxdflag=0;
        jxdflag=0;
        DELAY_MS(5);
  }
 
}

void tly()
{
  uint8 zq_error;
  uint8 zq_error_num;
  uint8 zout_qz;
  uart_rx_irq_en(UART2);                                 //开串口接收中断
  ZOUT= ((double)(ucStrAngle[1]<<8| ucStrAngle[0]))/32768.0*180;
  ZOUT=360-ZOUT;
  if(ZOUT==360)
    ZOUT=0;
  Site_t tly = {80,0};
  LCD_num_BC(tly,ZOUT,3, BLUE,RED);
 
  
  if(abs(ZOUT_LAST3-ZOUT)<=1)
    tly_num++;
  else
    tly_num=0;
  
  if(tly_num>=30)
  {
  tly_flag=1;
  if(tly_num>200)
    tly_num=100;
  }                     
  else
  {
  tly_flag=0;
 
  }

  ZOUT_LAST3=ZOUT_LAST2;
  ZOUT_LAST2=ZOUT_LAST;
  ZOUT_LAST=ZOUT;

 //uart_rx_irq_dis(UART2);
}

void  main()
{
 // uart_init(UART2,115200);                   //初始化串口(UART2 是工程里配置为printf函数输出端口，故已经进行初始化) 
 // set_vector_handler( UART2_RX_TX_VECTORn ,uart2_handler );   // 设置中断服务函数到中断向量表里
  
  
  lcd_camera_init();                      //摄像头液晶初始化
  
  ftm_pwm_init(FTM3, FTM_CH6,100,dj_mid);
  
  //key_init(KEY_U);
  
  gpio_init(PTE4,GPI,0);
  gpio_init(PTE5,GPI,0);
  gpio_init(PTE6,GPI,0);
  port_init_NoALT(PTE4, PULLUP );
  port_init_NoALT(PTE5, PULLUP );
  port_init_NoALT(PTE6, PULLUP );

  gpio_init(PTE0,GPI,0);
  gpio_init(PTE1,GPI,0);
  gpio_init(PTE2,GPI,0);
  gpio_init(PTE3,GPI,0);
  qd_flag=0;
  qd_num=0;
  while(qd_flag==0)
    qidong();
  dianji_init();
  while (1)
  {
   zhaodeng();
   //tly();
   duoji();
   
  }  
  
}
void zhaodeng()
{
  uint8 i,j,k;
  //int16 var=0;
  uint8 deng_x,deng_y;
  uint8 deng_up=0,deng_down=79 ;
  uint8 deng_up_flag,deng_down_flag;
  uint8 deng_left=0;
  uint8 deng_right=80;
  uint8 deng_right_flag;
  uint8 deng_left_flag;
  camera_get_img();
  lcd_display();
  img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);
  
  deng_x=0;
  deng_y=0;
  deng_up_flag=0;
  deng_down_flag=0;
  
  for(i=8;i<=55;i++)/*xiugai   */
  {
      for(j=78;j>3;j--)
      {
        if((i>18&&img[i][j]==0x00 && img[i][j-1]==0x00 && img[i][j-2]==0xff && img[i][j-3]==0xff)
           ||(i<=18&&img[i][j]==0x00 && img[i][j-1]==0x00 && img[i][j-2]==0xff)
             )
        {
        deng_right=j-2;
        deng_right_flag=1;
        break;
        }
        else
        {
          deng_right_flag=0;
        }
          
      }
       for(j=1;j<77;j++)
      {
        if((i>18&&img[i][j]==0x00 && img[i][j+1]==0x00 && img[i][j+2]==0xff && img[i][j+3]==0xff)
      ||(i<=18&&img[i][j]==0x00 && img[i][j+1]==0x00 && img[i][j+2]==0xff))
        {
        deng_left=j+2;
        deng_left_flag=1;
        break;  
        }
        else
        {
         deng_left_flag=0;
        }
      }
      
      deng_i=i;
      deng_j=(deng_left+deng_right)/2; 
      //if(i>=20)/* xiugai   反光是否存在*/
       //twenty();
      
      if(deng_right_flag==1 && deng_left_flag==1)
      {
          deng_x=deng_right-deng_left+1;
          
          for(k=deng_i;k>1;k--)
          {
            if(img[k][deng_j]==0xff&&img[k-1][deng_j]==0x00&&img[k-2][deng_j]==0x00)
            {
              deng_up=k-1;/*xiugai*/
              deng_up_flag=1;
              break;
            }
            else
            {
              deng_up_flag=0;
            }
          }
           for(k=deng_i;k<57;k++)
          {
            if(img[k][deng_j]==0xff&&img[k+1][deng_j]==0x00&&img[k+2][deng_j]==0x00)
            {
              deng_down=k;
              deng_down_flag=1;
              break;
            }
            else
            {
              deng_down_flag=0;
            }
          }
          if(deng_up_flag==1 && deng_down_flag==1)
          {
           
            deng_y=deng_down-deng_up;/*xiugai*/
            deng_i=(deng_up+deng_down)/2;
            for(k=deng_j;k<78;k++)
            {
              if(img[deng_i][k]==0xff&&img[deng_i][k+1]==0x00&&img[deng_i][k+2]==0x00)
              {
                deng_right=k;
                deng_right_flag=1;
                break;
              }
              else
                deng_right_flag=0;
            }
            for(k=deng_j;k>0;k--)/*xiugai*/
            {
              if(img[deng_i][k]==0x00&&img[deng_i][k+1]==0x00&&img[deng_i][k+2]==0xff)
              {
                deng_left=k+2;
                deng_left_flag=1;
              }
                else
                  deng_left_flag=0;
            }
              
                deng_x=deng_right-deng_left+1;
          /*   xiugai   限制判断为灯的条件，必须修改*/
                if( (deng_i>=8&&deng_i<18&&deng_x<6&&deng_y<15)
                 ||(deng_i>=18&&deng_i<26&&deng_x<12&&deng_y<20&&deng_x>4)
                   ||(deng_i>=26&&deng_x>8&&deng_y>8))
                {
                  dengflag1=1;
                  break;
                }
                else
                  dengflag1=0;
              
          }
          else 
            dengflag1=0;
      }
      else
        dengflag1=0;
  }
  
  if(dengflag1==0)
  {
    num++;
    Site_t o = {40,80};
    LCD_num_BC(o,num,3, BLUE,RED);
    if(num>500)
      num=50;
  }
  else
  {
    num=0;
    
    Site_t hhh = {20,60};
    LCD_num_BC(hhh,deng_y,2, BLACK,RED);
  }
  
  if(num>5)                             //shujv 
    dengflag=0;
  else
  {
    dengflag=1;
    /*if(gdflag==1)
      dengflag=0;
    else
      dengflag=1; */
  }

   
   if(dengflag1==0)                                            //丢灯以上次的算
  {
    deng_i=deng_i_last;
    deng_j=deng_j_last;
  }

   
  deng_i_last=deng_i;
  deng_j_last=deng_j;
  

  
  if(dengflag==1)
  {
    flaglcd_display();
  }
  Site_t abc = {60,100};
  LCD_num_BC(abc,dengflag,1, BLUE,RED);
}


void raodeng()
{
  if(dengflag==1&&deng_i>40)
  {
    raodengflag=1;
  }
  else
    raodengflag=0;
  if(raodengflag==0)
  {
    ;
  }
}
    
void gdkg()
{
  double gdkg1=0,gdkg2=0,gdkg3=0;
 
  uint8 gdkg_num,gdkg1_num,gdkg2_num,gdkg3_num;
  gdkg1=gpio_get (PTE6);
   gdkg2=gpio_get (PTE5);
   gdkg3=gpio_get (PTE4);
   
   Site_t x = {0,100};
   LCD_num_BC(x,gdkg1,2, BLUE,RED);
    Site_t y = {20,100};
   LCD_num_BC(y,gdkg2,2, BLUE,RED);
    Site_t z = {40,100};
   LCD_num_BC(z,gdkg3,2, BLUE,RED);
   
   
    if(gdkg1==0||gdkg2==0||gdkg3==0)
   {
     gdkg_error_num++;
   }
   else
      gdkg_error_num=0;
   if(gdkg_error_num>=30)
   {
     gdkg_error_flag=1;
   }
     
   else
     gdkg_error_flag=0;
   
   
   if(gdkg1==1&&gdkg2==1&&gdkg3==1)
   {
     zhangai=0;
   }
   else
   {
     gdkg1_num=0;gdkg2_num=0;gdkg3_num=0;
     for(gdkg_num=0;gdkg_num<2;gdkg_num++)
     {
       gdkg1=gpio_get (PTE6);
       gdkg2=gpio_get (PTE5);
       gdkg3=gpio_get (PTE4);    
       if(gdkg1==0)
         gdkg1_num++;
       if(gdkg2==0)
         gdkg2_num++;
       if(gdkg3==0)
         gdkg3_num++;
     }
     if(gdkg1_num>=1)
     {
       gdkg1=1;
     }
     else
       gdkg1=0;
     if(gdkg2_num>=1)
     {
       gdkg2=1;
     }
     else
       gdkg2=0;
     if(gdkg3_num>=1)
     {
       gdkg3=1;
     }
     else
       gdkg3=0;
       
   if(gdkg1==1&&gdkg2==1&&gdkg3==1)
   {
     zhangai=4;
   }
  else  if(gdkg1==1&&gdkg2==1&&gdkg3==0)
   {
     zhangai=4;
   }
  else  if(gdkg1==1&&gdkg2==0&&gdkg3==1)
   {
     zhangai=5;
   }
  else  if(gdkg1==1&&gdkg2==0&&gdkg3==0)
   {
    zhangai=3;
   }
   else if(gdkg1==0&&gdkg2==1&&gdkg3==1)
   {
    zhangai=-4;
   }
  else  if(gdkg1==0&&gdkg2==1&&gdkg3==0)
   {
     zhangai=4;
   }
   else if(gdkg1==0&&gdkg2==0&&gdkg3==1)
   {
    zhangai=-3;
   }
   }
  
    //Site_t gdkg_n = {80,40};
    //LCD_num_BC(gdkg_n, gdkg_error_num,4, BLACK,RED);
}


void flaglcd_display()
{
  if(deng_i>0&&deng_i<59&&deng_j>0&&deng_j<79)
  {
    Site_t site = {deng_j,deng_i};
    LCD_cross (site,40,RED); 
    //显示十字线中间点的坐标
      Site_t site_shizidian_x = {0,80};
      LCD_num_BC(site_shizidian_x,deng_i,2, BLUE,RED);
      Site_t site_shizidian_y = {20,80};
      LCD_num_BC(site_shizidian_y,deng_j,2, BLUE,RED);
  } 
}


/******************************液晶摄像头初始化*************************************/
void lcd_camera_init()
{
    LCD_init();  
    Site_t hh = {0,60};
    LCD_num_BC(hh,2,2, BLACK,RED);//LCD_init
    camera_init(imgbuff);

    //配置中断服务函数
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
}

/******************************电机调速初始化*************************************/
void dianji_init()
{
 // uart_init(UART4,38400);
    pit_init_ms(PIT0,10);                                 //初始化PIT0，定时时间为： 10ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler2);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    enable_irq (PIT0_IRQn);  
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);      //初始化 电机 PWM      
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,0);      //初始化 电机 PWM      
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
}

void lcd_display()
{
    Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size;                   //显示区域图像大小

    

    size.H = 60;
    size.W = 80; 
    LCD_Img_Binary_Z(site, size, imgbuff, imgsize); 
   
}

void uart2_handler(void)
{
    char ch;
    if(uart_query(UART2) == 1)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar(UART2, &ch);                   //无限等待接受1个字节
        Re_buf[counter]=ch;
        if(counter==0&&Re_buf[0]!=0x55) return;//第0号数据不是帧头
	    counter++; 
	    if(counter==11)             //接收到11个数据
	    {    
	       counter=0;               //重新赋值，准备下一帧的接收      
			switch(Re_buf[1])
			{                
			//case 0x51:ucStrAngle[2]=Re_buf[2];ucStrAngle[3]=Re_buf[3];
                         //ucStrAngle[4]=Re_buf[4];ucStrAngle[5]=Re_buf[5];break;
                        //case 0x55:ucStrAngle[0]=Re_buf[4];ucStrAngle[1]=Re_buf[5];break;
                       case 0x53:ucStrAngle[0]=Re_buf[6];ucStrAngle[1]=Re_buf[7];break;
                       // case 0x54:ucStrAngle[4]=Re_buf[6];ucStrAngle[5]=Re_buf[7];break;
			}  
            }
    }
}
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

void PIT0_IRQHandler2(void)//电机放在中断里
{
   if(PTE2_IN==1 && PTE3_IN==1 && PTE0_IN==1) 
   {
        if( gdkg_error_flag==1)
        {
          dianji_fanzhuan();
        }
        else if(dj_left_stop==0&&dj_right_stop==0)
        {
          if(zqflag==1)
         {
           dianji_chasu();
         }
         else if(yxdflag==1)
         {
           dianji_gaosu();
         }
         else if(jxdflag==1)
         {
           dianji_zhongsu();
         }
         else if(ybzflag==1)
         {
           dianji_zhongsu();
         }
          else if(jbzflag==1)
         {
           dianji_chaodisu();
         }
          else if(gdflag==1)
         {
           dianji_disu();
         }
        }
   }
   
    if(PTE2_IN==1 && PTE3_IN==1 && PTE0_IN==0) 
      dianji_buzhuan();
Site_t sdz = {80,20};
LCD_num_BC(sdz,sudu,4, BLUE,RED);
 PIT_Flag_Clear(PIT0);       //清中断标志位

}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

void twenty()
{
  uint8 i,j;
  uint8 deng_i1_sb,deng_i2_xb;
  uint8 flag;
  uint8 t;
  t=0;
  flag=0;
  
  for(i=deng_i,j=deng_j;i>11;i--)
  {
    if(img[i][j]==0xff && img[i-1][j]==0x00)
    {
      deng_i1_sb=i;
      flag=1;
      t=deng_i-deng_i1_sb+1;
      break;
    }
    else
    {
      flag=0;
    }
  }
  
  if(flag==1)
  {
    for(;i>10;i--)
    {
      if(img[i][j]==0x00 && img[i-1][j]==0xff && img[i-2][j]==0xff)
      {
        deng_i2_xb=i-1;
        break;
      }
    }
  }
  else
    deng_i2_xb=0;

  if(deng_i1_sb-deng_i2_xb<=4)
    deng_i=deng_i2_xb+1;
  if(t>6)
    deng_i=(deng_i+deng_i1_sb)/2-1;

}
/******************************电机驱动不转*************************************/        
void dianji_buzhuan()
{
  sudu=0;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) =sudu;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = 0;//左
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = 0;//右             
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) =sudu;
}


/******************************电机驱动反转*************************************/        
void dianji_fanzhuan()
{
  sudu=1300;           // baodi
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) =sudu;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = 0;//左
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = 0;//右             
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) =sudu;
}



/******************************电机驱动超低速*************************************/        
void dianji_chaodisu()
{
  sudu=1750;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) = 0;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = sudu;//左
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = sudu;//右             
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) = 0;
}
/******************************电机驱动差速*************************************/        
void dianji_chasu()
{
  sudu=2333;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) = 0;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = 1950;//     1900you
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = 1200;//        1000zuo            
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) = 0;
}


/******************************电机驱动低速*************************************/        
void dianji_disu()
{
  sudu=1800;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) = 0;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = sudu;//左
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = sudu;//右             
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) = 0;
}


/******************************电机驱动中速*************************************/        
void dianji_zhongsu()
{
  sudu=2000;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) = 0;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = sudu;//左
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = sudu;//右   
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) = 0;
}

/******************************电机驱动高速*************************************/        
void dianji_gaosu()
{
  sudu=2200;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM) = 0;                  
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM) = sudu;//左
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM) = sudu;//右                 
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM) = 0;
}


void qidong()
{

  
  camera_get_img();
  lcd_display();
  img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);
  
  uint8 i,j;
  uint8 qd_right,qd_right_flag;
  uint8 qd_left,qd_left_flag;
  qd_left=0;
  qd_right=79;
  qd_left_flag=0;
  qd_right_flag=0;
  for(i=50;i>=10;i--)/*xiugai  */
  {
    for(j=78;j>=2;j--)
    {
      if(img[i][j]==0x00 && img[i][j-1]==0xff && img[i][j-2]==0xff)
      {
        qd_right=j-1;
        qd_right_flag=1;
        break;
      }
      else
      {
        qd_right_flag=0;
      }
    }
    for(j=78;j>=2;j--)
    {
      if(img[i][j]==0xff && img[i][j-1]==0xff && img[i][j-2]==0x00)
      {
        qd_left=j-1;
        qd_left_flag=1;
        break;
      }
      else
      {
        qd_right_flag=1;
      }
    }
    if(qd_right_flag==1 && qd_left_flag==1 )
    {
      qd_num++;
    }
    else
    {
      qd_num=0;
    }
    if(qd_num>=5)
    {
      qd_flag=1;
      qd_num=0;
      break;
    }
    else
    {
      qd_flag=0;
    }
  }
}


void tly_duoji()
{
  duty=dj_right_max;
  ftm_pwm_duty(FTM3 ,FTM_CH6,duty);
  DELAY_MS(1000);
  tly_flag=0;
  tly_num=0;

}

void gdkg_duoji()
{
  if(duty>dj_mid)
    duty=dj_right_max;
  else
    duty=dj_left_max;
  ftm_pwm_duty(FTM3 ,FTM_CH6,duty);
  DELAY_MS(1000);
   gdkg_error_flag=0;
    gdkg_error_num=0;

}
void duoji_xuanze()
{
   if( gdkg_error_flag==0)
    duoji();
   else
    gdkg_duoji();

}