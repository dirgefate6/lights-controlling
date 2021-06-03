#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "string.h"
#include "ov7670.h"
#include "ov7725.h"
#include "tpad.h"
#include "timer.h"
#include "exti.h"
#include "usmart.h"
#include "malloc.h"
#include "sdio_sdcard.h" 
#include "stmflash.h"
#include "adc.h"
#include "lsens.h"
#include "w25qxx.h"	
//************************************************/
/*程序名称：教室智能照明控制系统主程序代码
 硬件设备：STM32F103ZET6  ov7725图像传感器
*/
//************************************************/
//传感器名字宏定义
#define  OV7725 1
#define  OV7670 2
//由于OV7725传感器安装方式原因,OV7725_WINDOW_WIDTH相当于LCD的高度，OV7725_WINDOW_HEIGHT相当于LCD的宽度
//注意：此宏定义只对OV7725有效
#define  OV7725_WINDOW_WIDTH		320 // <=320
#define  OV7725_WINDOW_HEIGHT		240 // <=240
#define FLASH_SAVE_ADDR    0X0802D000   //彩色图像数据FLASH操作入口地址1 160k空间，用于存储一帧图像
#define FLASH_SAVE_ADDR2    0X08055000  //彩色图像数据FLASH操作入口地址2 160K空间，用于才形态学处理过程中存入中间变量
const u8*LMODE_TBL[6]={"Auto","Sunny","Cloudy","Office","Home","Night"};//6种光照模式	    
const u8*EFFECTS_TBL[7]={"Normal","Negative","B&W","Redish","Greenish","Bluish","Antique"};	
//7种特效 
//外部变量定义
extern u8 ov_sta;	//在exit.c里 面定义
extern u8 ov_frame;	//在timer.c里面定义 
u32 b_flag =0;       
u8 people_flag=0x00;    //人员位置标识，用一个字节的低六位分别表示教室六个区域是否存在人，1表示此区域有人，0表示此区域没人
u8 flag1_w=0;     //每个区域分别用一个标识符表示是否有人
u8 flag2_w=0;
u8 flag3_w=0;
u8 flag4_w=0;
u8 flag5_w=0;
u8 flag6_w=0;
/////////////////////////////////
//bk_refresh()函数用于将获取到的教室无人的背景图像存入到外扩flash 中、
//括号中的参数表示一幅背景图像开始存储的位置
/////////////////////////////////
void bk_refresh(u8 address_flag)
{
	u32 i,j;
	u8 picture1[1024]={0};  //申请一个1024大小的数组，用于写入flash时存储像素数据
 	u16 color;
  u16 camera_temp;
  u8 camera_red;
  u8 camera_green;
  u8 camera_blue;	
	u8 camera_gray;
	//////////////change   
	u32 buf_flag = 0;
	u8 t=0;	
	u8 buf2[1]={0xFF};
	///////////////change
	if(ov_sta)//检测是否有帧中断，更新
	{
	LCD_Scan_Dir(U2D_L2R);		//设置lCD显示屏的扫描方式为，从上到下,从左到右		 
LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//将显示区域设置到屏幕中央
		if(lcddev.id==0X1963)		LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//将显示区域设置到屏幕中央
		LCD_WriteRAM_Prepare();     //开始写入LCD GRAM	
		OV7725_RRST=0;				//开始复位读指针 
		OV7725_RCK_L;         
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//复位读指针结束 
		OV7725_RCK_H; 
	
		for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
		{
			for(j=0;j<OV7725_WINDOW_WIDTH;j++)
			{
				OV7725_RCK_L;
				color=GPIOC->IDR&0XFF;	//读数据
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=GPIOC->IDR&0XFF;	//读数据
				OV7725_RCK_H; 
				//分别读取两个字节的像素数据并存入到16位无符号整数color中
			  ////////////////////////////////////////////
	              //rgb565 彩色图像灰度化 分别变成r g  b 三个分量
				camera_red=(color&0xF800)>>8;
                   camera_green=(color&0x07E0)>>3;
                   camera_blue=(color&0x001F)<<3;				
				camera_gray=(30*camera_red+59*camera_green+11*camera_blue+50)/100;	
//	采用加权平均值 灰度化		
				////////////////////////////////////////////////
				picture1[(buf_flag% 1024 )] = camera_gray;   
//每读取1024个像素数据就存入到数组中去
				if(buf_flag%1024==1023)
        {
      	W25QXX_Write(picture1,(address_flag+1)*76800+((buf_flag/1024) * 1024),1024);			
//向w25q128 存入背景图像数据，一帧图像有76800KB，所以存入图像的时候要在基础的地址上加上76800×背景编号，方便存取。
		}			
		buf_flag ++;
		LCD->LCD_RAM=color;  
			}
		}
 		ov_sta=0;					//清零帧中断标记
		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
	} 
}

//此函数用于计算教室灰度化后的实时图像和背景灰度图像进行帧差，计算两者之间差异的大小
//根据设定阈值将图像分成0 和1的像素，方便之后的形态学处理，即进行灰度图像二值化

void cf(void)
{
	u16 picture1[1024]={0};   // 用于从flash中读写数据时候的数组
	u8 picture2[1024]={0};//用于从为5去28中读写数据时候的数组
	u32 i,j;
     for(i=0;i<75;i++)     //一帧320×240的图像 需要75KB， 一次读取1KB，所以需要循环75次
		{
		STMFLASH_Read(FLASH_SAVE_ADDR+(i * 2048),picture1,1024);  
//FLASH_ADDR 为flash存取的基准地址，因为一个像素占两个字节，所以需要乘以2048
		W25QXX_Read(picture2,i*1024,1024);//从w25q128读取背景图像
		for(j=0;j<1024;j++)         //对每一次数组中每个像素进行差分比较
		{
		if(picture1[j]>picture2[j])  
//因为不知道灰度值哪个大，所以对两个数值要先进行比较，用大的减去小的
		{
	      picture1[j] = picture1[j]-picture2[j];
		if (picture2[j] <30) picture1[j]=0;   
//如果两者之间的灰度差大于等于30，就设置此像素值为1，否则设置此像素值为0
		else picture1[j]=1;
		}
		else
         {
		 picture1[j] = picture2[j]-picture1[j];
			if (picture2[j] <30) picture1[j]=0;
			else picture1[j]=1;
		}
		}
		STMFLASH_Write(FLASH_SAVE_ADDR+(i * 2048),picture1,1024);
			
		}
}


//remo_noise（）函数用于去除二值化后的图像中的噪声，这些噪声一般都比较小，所以先进行腐蚀，再
//进行膨胀操作
//使用4个不同的算子，分别去除不同方向上的噪声
void remo_noise(void)
{ 
u32 i,j,k;
u16 picture1[960]={0};
u16 picture2[320]={0};
  u16 picture3[320]={0};
///////////////////////////////
//先进行竖直方向上的腐蚀和膨胀操作，即“开”处理
	for (i=0;i<238;i++)  //腐蚀
{
STMFLASH_Read(FLASH_SAVE_ADDR+i*320,picture1,960);
//样采用循环读取，一次读取三行所以是960个数据 总共需要读取238次，第一行和最后一行忽略
	for(k=0;k<320;k++)
	{
	 if(picture1[320+k]==1 &&picture1[k]==1  && picture1[640+k]==1)picture2[k]=1;			
	//腐蚀操作		
	else picture2[k]=0;
	}			
STMFLASH_Write(FLASH_SAVE_ADDR2+(i+1)*320,picture2,320);	 
//循环存入flash中，因为只有中间一行有用，所以只需要存入320个数据
}
//
for (i=1;i<239;i++)  //膨胀，同腐蚀操作，只不过进行逆操作
	{
     STMFLASH_Read(FLASH_SAVE_ADDR2+i*320,picture2,320);
		 for(k=0;k<320;k++)
	{
		if(picture2[k]==1)
		{
		   picture1[k]=1;
            picture1[320+k]=1;	
            picture1[640+k]=1;						
		}
	}			
STMFLASH_Write(FLASH_SAVE_ADDR+(i-1)*320,picture1,960);	 
}
/////////////////////////////////////////////
//进行水平方向上的腐蚀和膨胀操作，

	for (i=0;i<240;i++)
	{
	  STMFLASH_Read(FLASH_SAVE_ADDR+i*320,picture3,320);
		 for(k=1;k<319;k++)
				{
				  if(picture3[k-1]==1 &&picture3[k]==1  && picture3[k+1]==1)picture2[k]=1;		
					else picture2[k]=0;
				}			
		 STMFLASH_Write(FLASH_SAVE_ADDR2+i*320,picture2,320);	 
}
for (i=0;i<240;i++)
	{	
     STMFLASH_Read(FLASH_SAVE_ADDR2+i*320,picture3,320);
		 for(k=1;k<319;k++)
	{
		if(picture3[k]==1)
		{
			picture2[k-1]=1;
            picture2[k]=1;	
            picture2[k+1]=1;						
		}
	}			
		 STMFLASH_Write(FLASH_SAVE_ADDR+i*320,picture2,320);	 
}
/////////////////////////////////////////////////////////////////
//进行右上45度方向上的腐蚀和膨胀操作，
	for (i=0;i<238;i++)
	{
	  STMFLASH_Read(FLASH_SAVE_ADDR+i*320,picture1,960);
		 for(k=1;k<319;k++)
				{
			if(picture1[320+k]==1 &&picture1[k+1]==1  && picture1[640+k-1]==1)picture2[k]=1;	
			else picture2[k]=0;
				}			
		 STMFLASH_Write(FLASH_SAVE_ADDR2+(i+1)*320,picture2,320);	 
}
for (i=1;i<239;i++)
	{	
     STMFLASH_Read(FLASH_SAVE_ADDR2+i*320,picture2,320);
		 for(k=1;k<319;k++)
				{
				  if(picture2[k]==1)
					{
						picture1[k+1]=1;
            picture1[320+k]=1;	
            picture1[640+k-1]=1;						
					}
				}			
		 STMFLASH_Write(FLASH_SAVE_ADDR+(i-1)*320,picture1,960);	 
}
////////////////////////////////////////
//进行左上45度方向上的腐蚀和膨胀操作，
	for (i=0;i<238;i++)
	{	
	  STMFLASH_Read(FLASH_SAVE_ADDR+i*320,picture1,960);
		 for(k=1;k<319;k++)
				{
		 if(picture1[320+k]==1 &&picture1[k-1]==1  && picture1[640+k+1]==1)picture2[k]=1;		
		else picture2[k]=0;
				}			
		 STMFLASH_Write(FLASH_SAVE_ADDR2+(i+1)*320,picture2,320);	 
}
	

for (i=1;i<239;i++)
	{
     STMFLASH_Read(FLASH_SAVE_ADDR2+i*320,picture2,320);
		 for(k=1;k<319;k++)
				{
				  if(picture2[k]==1)
					{
						picture1[k-1]=1;
            picture1[320+k]=1;	
            picture1[640+k+1]=1;						
					}
				}			
		 STMFLASH_Write(FLASH_SAVE_ADDR+(i-1)*320,picture1,960);	 
}
}


// r1eg_person()函数用于对于图像中的各个区域检测是否有人存在
//
void recg_person(void)
{
u32 i,j,k,m;
	u16 picture1[1920]={0};//循环读取，一次读取6行，总共1920个数据
	u32 coment1=0;   //用于计算6个区域中每个6×10大小图像块中1的个数
	u32 coment2=0;	
	u32 coment3=0;	
	u32 coment4=0;	
	u32 coment5=0;	
	u32 coment6=0;
	u8 flag1=0;   //用于标记6个区域内是否有人
	u8 flag2=0;
	u8 flag3=0;
	u8 flag4=0;
	u8 flag5=0;
	u8 flag6=0;	
////////////////////////////
//区域1和区域2 的	人员识别
  for(i=0;i<16;i++)    // 由于区域一和区域2在图像中所占区域大约占50行，每次需要读取6×10大小的图像块，并且间隔三行来读取所以需要循环16次
	{	
		STMFLASH_Read(FLASH_SAVE_ADDR+(i * 960),picture1,1920);
//一次读取6行，所以需要读取1920个数据
	  for (j=0;j<31;j++)
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment1++;   
 //计算每一个6×10大小的方块中像素为1的个数。
				  } 
			}
		if(coment1>55) flag1=1;   
//如果这个个数大于55，则认为此区域有人，令flag1等于1 并且跳出循环
		if(flag1==1)  break;
		}
		for (j=32;j<63;j++)   //针对区域2进行同样的操作
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment2++;
				  } 
			}
		if(coment2>55) flag2=1; 
		if(flag2==1)  break;
		}
		if(flag1==1 && flag2==1)break;
	}
///////////////////////////////////////	
	for(i=0;i<13;i++)   //区域三和区域四约占40行，所以需要循环13次 其余原理同上
	{	
		STMFLASH_Read(FLASH_SAVE_ADDR+16000+(i * 960),picture1,1920);
	  for (j=0;j<31;j++)
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment3++;
				  } 
			}
		if(coment3>55) flag3=1; 
		if(flag3==1)  break;
		  
		
		}
		for (j=32;j<63;j++)
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment4++;
				  } 
			}
		if(coment4>55) flag4=1; 
		if(flag4==1)  break;	
		}
		if(flag3==1 && flag4==1)break;
	}
/////////////////////////////////////////////////////
for(i=0;i<10;i++)//区域5和区域6约占30行，所以需要循环10次 其余原理同上
	{
		    printf("  rp44444ok");
		STMFLASH_Read(FLASH_SAVE_ADDR+28800+(i * 960),picture1,1920);
	  for (j=0;j<31;j++)
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment5++;
				  } 
			}
		if(coment5>55) flag5=1; 
		if(flag5==1)  break;
		}
		for (j=32;j<63;j++)
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment6++;
				  } 
			}
		if(coment6>55) flag6=1; 
		if(flag6==1)  break;

		}
		if(flag5==1 && flag6==1)break;	
	}	
	//////////////////////////
	flag1_w=flag1;
	flag2_w=flag2;
	flag3_w=flag3;
	flag4_w=flag4;
	flag5_w=flag5;
	flag6_w=flag6;
	//将局部变量传递给全局变量
}
//更新LCD显示(OV7725图像传感器，系统使用此传感器)
//并且此过程将获取到的图像信息直接进行灰度化存储到flash中
void OV7725_camera_refresh(void)
{
	u32 i,j;
	u16 picture2[1024]={0};    //中间存储图像数据数组
 	u16 color;                //从ov7725读取的彩色像素存入变量
    u16 camera_temp;
    u8 camera_red;     //R G B 三个分量  以及灰度像素分量
    u8 camera_green;
    u8 camera_blue;	
	u8 camera_gray;
	//////////////change
	u32 brightness1 = 0;    //计算图像中两块50×50大小的灰度图像平均值
	u32 buf_flag = 0;      //灰度值是否大于阈值的标识，1标识大于，0标识不大于
	u8 t=0;	
	u8 buf2[1]={0xFF};  
	///////////////change
	if(ov_sta)//ov7725有帧中断更新
	{
		LCD_Scan_Dir(U2D_L2R);		//从上到下,从左到右
	LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//将显示区域设置到屏幕中央
		if(lcddev.id==0X1963)
	LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//将显示区域设置到屏幕中央
		LCD_WriteRAM_Prepare();     //开始写入GRAM	
		OV7725_RRST=0;				//开始复位读指针 
		OV7725_RCK_L;
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//复位读指针结束 
		OV7725_RCK_H; 
		for(i=0;i<OV7725_WINDOW_HEIGHT;i++)   //总共有240行
		{
			for(j=0;j<OV7725_WINDOW_WIDTH;j++)  //每行有320个像素
			{
				OV7725_RCK_L;
				color=GPIOC->IDR&0XFF;	//读数据
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=GPIOC->IDR&0XFF;	//读彩色图像数据，标识到color中
				OV7725_RCK_H; 
			  ////////////////////////////////////////////
	camera_red=(color&0xF800)>>8;       //提取三个分量计算灰度值
     camera_green=(color&0x07E0)>>3;
     camera_blue=(color&0x001F)<<3;		
	camera_gray=(30*camera_red+59*camera_green+11*camera_blue+50)/100;  
//计算两个50×50大小的灰度平均值，行从160到209，列从40到90,20到299
if((i>=160 && i<210 && j>=40 &&j<90)||(i>=160 && i<210 && j>=250 &&j<300))
	{
	      brightness1 =brightness1 + camera_gray;
				}
	////////////////////////////////////////////////
	//	将灰度像素数据每隔1024个就存入到flash中
		picture2[(buf_flag% 1024 )] = camera_gray;
		if(buf_flag%1024==1023)
        {
		STMFLASH_Write(FLASH_SAVE_ADDR+((buf_flag/1024) * 2048),picture2,1024);
				
		}			
		buf_flag ++;
				
//////////////////////////////////		
    camera_red=camera_gray;
    camera_green=camera_gray;
    camera_blue=camera_gray;
    camera_temp=(((camera_gray<<8)&0xf800)+((camera_gray<<3)&0x07e0)+((camera_gray>>3)&0x001f));
////////////////////////////////////////
LCD->LCD_RAM=color;  //将图像数据显示到LCD上
}
}
brightness1 = brightness1/5000;   //计算灰度平均值
if(brightness1 < 120)b_flag=1;    //看实时图像的灰度平均值是否大于阈值，大于阈值，光线强，不开灯，b_flag=0，反之，光线弱，要开灯，b_flag=1
else b_flag=0;
 	ov_sta=0;					//清零帧中断标记
		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//恢复默认扫描方向 
	} 
}
//////////////////////
//由于电磁继电器需要6输入，所以需要对PD11、PD12 PD13 PE0 PE1 PE2 的串口进行初始化，之后才能方便控制
void ele_mag_Init(void)
{
GPIO_InitTypeDef  GPIO_InitStructure;	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);	 
//使能PD,PE端口时钟
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;	 //PD端口的管教为11 12 13
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOD, &GPIO_InitStructure);	
////////////////////////////
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;	//PE端口的管脚为0 1 2
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);		//根据设定参数初始化GPIOB.5
  GPIO_ResetBits(GPIOD,GPIO_Pin_11);
GPIO_ResetBits(GPIOD,GPIO_Pin_12);
GPIO_ResetBits(GPIOD,GPIO_Pin_13);
  GPIO_ResetBits(GPIOE,GPIO_Pin_0);    //并将6个输出端口都设置为0，即关闭所有灯光
GPIO_ResetBits(GPIOE,GPIO_Pin_1);
GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	
}
//此函数的作用是将6个表示区域内是否有人的标识用一个字节表示，即将六个数用一个表示，每一位分别表示此区域是否有人
u8 six2one(u8 flag1,u8 flag2,u8 flag3,u8 flag4,u8 flag5,u8 flag6)
{
	u8 p_flag=0;
	p_flag=people_flag&0X00;
     p_flag +=flag1;
	p_flag<<=1;
	p_flag +=flag2;
	p_flag<<=1;
	p_flag +=flag3;
	p_flag<<=1;
	p_flag +=flag4;
	p_flag<<=1;
	p_flag +=flag5;
	p_flag<<=1;
	p_flag +=flag6;
	return p_flag;
	
}

//此函数用于将教室背景模板存入到w25q128芯片中去，采用6层循环
void update_bk(void)
{
  u32 i,j,k,m,n,o;
	u8 date_flag;
	for(i=0;i<2;i++)
	{
	  for(j=0;j<2;j++)
		{
		  for(k=0;k<2;k++)
			{
			  for(m=0;m<2;m++)
			{
			   for(n=0;n<2;n++)
			{
			   for(o=0;o<2;o++)
			{
			//首先对6个输出口进行设置，控制灯光的亮灭   
			if(i==1)GPIO_SetBits(GPIOD,GPIO_Pin_11);
			else GPIO_ResetBits (GPIOD,GPIO_Pin_11);
			if(j==1)GPIO_SetBits(GPIOD,GPIO_Pin_12);
			else GPIO_ResetBits (GPIOD,GPIO_Pin_12);
			if(k==1)GPIO_SetBits(GPIOD,GPIO_Pin_13);
			else GPIO_ResetBits (GPIOD,GPIO_Pin_13);
			if(m==1)GPIO_SetBits(GPIOE,GPIO_Pin_0);
			else GPIO_ResetBits (GPIOE,GPIO_Pin_0);
			if(n==1)GPIO_SetBits(GPIOE,GPIO_Pin_1);
			else GPIO_ResetBits (GPIOE,GPIO_Pin_1);
			if(o==1)GPIO_SetBits(GPIOE,GPIO_Pin_2);
			else GPIO_ResetBits (GPIOE,GPIO_Pin_2);
			date_flag=six2one(o,n,m,k,j,i);	
			delay_ms(1000);   //延迟1秒，保证图像稳定
			bk_refresh(date_flag) ;
				
			}
			}
			}
			}
		}	
	}
}
int main(void)   //主程序
 {	 
	 ////////
	 u8 *buf=0;
	 u8 t=0;
	 u8 b_buff=0X00;
	 u32 x_flag;
	 u8  updatebuff1=0;
	 u8  updatebuff2;
	 u8 j=0;
	 u8 picture4[1024]={0};
	 ////////////
	u8 sensor=0;
	u8 key;
 	u8 i=0;	    
	u8 msgbuf[15];//消息缓存区
	u8 tm=0;
	u8 lightmode=0,effect=0;
	s8 saturation=0,brightness=0,contrast=0;

	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为 115200
 	usmart_dev.init(72);		//初始化USMART		
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD  
	W25QXX_Init();			//W25QXX初始化
	TPAD_Init(6);				//触摸按键初始化 
	ele_mag_Init();     //电磁继电器输出端口初始化
 	POINT_COLOR=RED;			//设置字体为红色 
	LCD_ShowString(30,50,200,16,16,"Classroom lighting control");	
	LCD_ShowString(30,70,200,16,16,"OV7725 TEST");	
	LCD_ShowString(30,130,200,16,16,"    ");
	LCD_ShowString(30,150,200,16,16,"    ");
	LCD_ShowString(30,170,200,16,16,"    ");
	LCD_ShowString(30,190,200,16,16,"    ");	 
  LCD_ShowString(30,210,200,16,16,"OV7725 Init...");	 	
		
while(W25QXX_ReadID()!=W25Q128)			//检测不到W25Q128
	{
		LCD_ShowString(30,270,200,16,16,"W25Q128 Check Failed!");
		delay_ms(500);
		LCD_ShowString(30,290,200,16,16,"Please Check!        ");
		delay_ms(500);
		LED0=!LED0;//DS0闪烁
	}
	LCD_ShowString(30,290,200,16,16,"W25Q128 Ready!");    
	/////////////////////change
	while(1)//初始化OV7725
	{
		if(OV7725_Init()==0)
		{
			sensor=OV7725;
			LCD_ShowString(30,270,200,16,16,"OV7725 Init OK  ");
			while(1)
			{
				key=KEY_Scan(0);
				if(key==KEY0_PRES)
				{
					 OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,0);//QVGA模式输出
			    OV7725_Light_Mode(lightmode);    //设置ov7725参数
			    OV7725_Color_Saturation(saturation);
			    OV7725_Brightness(brightness);
			    OV7725_Contrast(contrast);
			    OV7725_Special_Effects(effect);
			    OV7725_CS=0;
			    TIM6_Int_Init(10000,7199);	//10Khz计数频率,1秒钟中断					  
	        EXTI8_Init();						//使能外部中断8,捕获帧中断			
	        LCD_Clear(BLACK);	//清空屏幕
					break;
				}
				else if(key==KEY1_PRES)
				{
				OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,0);
			  	OV7725_Light_Mode(lightmode);
			    OV7725_Color_Saturation(saturation);
			    OV7725_Brightness(brightness);
			    OV7725_Contrast(contrast);
			    OV7725_Special_Effects(effect);
			    OV7725_CS=0;
				TIM6_Int_Init(10000,7199);			//10Khz计数频率,1秒钟中断			
	              EXTI8_Init();						//使能外部中断8,捕获帧中断		
	              LCD_Clear(BLACK);	//清空屏幕
				update_bk();
				break;
				}
				
			}				
			
			break;
		}else if(OV7670_Init()==0)
		{
			sensor=OV7670;
			LCD_ShowString(30,210,200,16,16,"OV7670 Init OK       ");
			delay_ms(1500);	 	   
			OV7670_Light_Mode(lightmode);
			OV7670_Color_Saturation(saturation);
			OV7670_Brightness(brightness);
			OV7670_Contrast(contrast);
			OV7670_Special_Effects(effect);
			OV7670_Window_Set(12,176,240,320);//设置窗口
			OV7670_CS=0;
			
			break;
		}else
		{
			LCD_ShowString(30,210,200,16,16,"OV7725 Error!!");
			delay_ms(200);
			LCD_Fill(30,210,239,246,WHITE);
			delay_ms(200);
		}
	}
	
	/////////////////////////
	OV7725_camera_refresh();  //首先获取一帧图像，计算特定区域的灰度平均值
	if(b_flag==1)   //b_flag等于1说明需要开灯
	{
	GPIO_SetBits(GPIOD,GPIO_Pin_11);  //6组灯光全部打开,GPIO输出高电平1
	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	GPIO_SetBits(GPIOD,GPIO_Pin_13);
     GPIO_SetBits(GPIOE,GPIO_Pin_0);
	GPIO_SetBits(GPIOE,GPIO_Pin_1);
	GPIO_SetBits(GPIOE,GPIO_Pin_2);
	for (x_flag=0;x_flag<60;x_flag++)
		{
		delay_ms(1000);//延迟 60s  ，供学生选择座位
		}
for(j=0;j<75;j++)   
//更新背景灰度图像，将教室灯光全开时的灰度图像存储到w25q128最开始的位置，方便进行差分
{
W25QXX_Read(picture4,64*76800+j*1024,1024);
W25QXX_Write(picture4,j * 1024,1024);		
}
		
	}
	else
	{
	for(j=0;j<75;j++)   //更新背景灰度图像，将教室灯全关时候灰度图像存储到w25q128最开始的位置，方便进行差分
	{
	W25QXX_Read(picture4,76800+j*1024,1024);
	W25QXX_Write(picture4,j * 1024,1024);		
	}
	}
	///////////////////////
 	while(1)
	{	
		
		if(TPAD_Scan(0))//检测到触摸按键 
		{	
		  update_bk();  //更新背景图像
		}
		if(sensor==OV7725)OV7725_camera_refresh();		//ov7725更新显示，记录平均灰度值
//		else if(sensor==OV7670)OV7670_camera_refresh();	//更新显示
 		b_buff<<=1;   //左移一位，最低位变成0，最高位舍弃，最低位用于记录最新的灰度平均值，
		if (b_flag==1)b_buff+=1;  
		if(b_buff==0XFF)	  
 //如果b_buff为0XFF，说明连续8帧图像的灰度平均值都低于阈值，教室需要开灯
		{
		cf();     //灰度图像背景差分，并进行二值化
   		remo_noise();  //图像去噪，二值化形态学操作
		recg_person();		//检验个区域是否有人
	     updatebuff2=six2one(flag1_w,flag2_w,flag3_w,flag1_w,flag4_w,flag5_w);  
//计算对应背景图像存储的首地址
		if(updatebuff2!=updatebuff1)   //如果updatebuff2等于updatebuff1，则说明不需要更新背景图像，否则需要更新背景图像
			{
			if(flag1_w==1)GPIO_SetBits(GPIOD,GPIO_Pin_11);  //更改教室灯光
			else GPIO_ResetBits (GPIOD,GPIO_Pin_11);
			if(flag2_w==1)GPIO_SetBits(GPIOD,GPIO_Pin_12);
			else GPIO_ResetBits (GPIOD,GPIO_Pin_12);
			if(flag3_w==1)GPIO_SetBits(GPIOD,GPIO_Pin_13);
			else GPIO_ResetBits (GPIOD,GPIO_Pin_13);
			if(flag4_w==1)GPIO_SetBits(GPIOE,GPIO_Pin_0);
			else GPIO_ResetBits (GPIOE,GPIO_Pin_0);
			if(flag5_w==1)GPIO_SetBits(GPIOE,GPIO_Pin_1);
			else GPIO_ResetBits (GPIOE,GPIO_Pin_1);
			if(flag6_w==1)GPIO_SetBits(GPIOE,GPIO_Pin_2);
			else GPIO_ResetBits (GPIOE,GPIO_Pin_2);
			updatebuff1=updatebuff2;
		     for(j=0;j<75;j++)   
//更新背景灰度图像，将灰度背景图像存储到w25q128最开始的位置，方便进行差分
				{
				  W25QXX_Read(picture4,(updatebuff2+1)*76800+j*1024,1024);
				  W25QXX_Write(picture4,j * 1024,1024);		
				}
			}		
		}
		i++;
		if(i>=15)//DS0闪烁.
		{
			i=0;
			LED0=!LED0;
 		}
	}	   
}