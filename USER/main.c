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
/*�������ƣ�����������������ϵͳ���������
 Ӳ���豸��STM32F103ZET6  ov7725ͼ�񴫸���
*/
//************************************************/
//���������ֺ궨��
#define  OV7725 1
#define  OV7670 2
//����OV7725��������װ��ʽԭ��,OV7725_WINDOW_WIDTH�൱��LCD�ĸ߶ȣ�OV7725_WINDOW_HEIGHT�൱��LCD�Ŀ��
//ע�⣺�˺궨��ֻ��OV7725��Ч
#define  OV7725_WINDOW_WIDTH		320 // <=320
#define  OV7725_WINDOW_HEIGHT		240 // <=240
#define FLASH_SAVE_ADDR    0X0802D000   //��ɫͼ������FLASH������ڵ�ַ1 160k�ռ䣬���ڴ洢һ֡ͼ��
#define FLASH_SAVE_ADDR2    0X08055000  //��ɫͼ������FLASH������ڵ�ַ2 160K�ռ䣬���ڲ���̬ѧ��������д����м����
const u8*LMODE_TBL[6]={"Auto","Sunny","Cloudy","Office","Home","Night"};//6�ֹ���ģʽ	    
const u8*EFFECTS_TBL[7]={"Normal","Negative","B&W","Redish","Greenish","Bluish","Antique"};	
//7����Ч 
//�ⲿ��������
extern u8 ov_sta;	//��exit.c�� �涨��
extern u8 ov_frame;	//��timer.c���涨�� 
u32 b_flag =0;       
u8 people_flag=0x00;    //��Աλ�ñ�ʶ����һ���ֽڵĵ���λ�ֱ��ʾ�������������Ƿ�����ˣ�1��ʾ���������ˣ�0��ʾ������û��
u8 flag1_w=0;     //ÿ������ֱ���һ����ʶ����ʾ�Ƿ�����
u8 flag2_w=0;
u8 flag3_w=0;
u8 flag4_w=0;
u8 flag5_w=0;
u8 flag6_w=0;
/////////////////////////////////
//bk_refresh()�������ڽ���ȡ���Ľ������˵ı���ͼ����뵽����flash �С�
//�����еĲ�����ʾһ������ͼ��ʼ�洢��λ��
/////////////////////////////////
void bk_refresh(u8 address_flag)
{
	u32 i,j;
	u8 picture1[1024]={0};  //����һ��1024��С�����飬����д��flashʱ�洢��������
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
	if(ov_sta)//����Ƿ���֡�жϣ�����
	{
	LCD_Scan_Dir(U2D_L2R);		//����lCD��ʾ����ɨ�跽ʽΪ�����ϵ���,������		 
LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//����ʾ�������õ���Ļ����
		if(lcddev.id==0X1963)		LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��LCD GRAM	
		OV7725_RRST=0;				//��ʼ��λ��ָ�� 
		OV7725_RCK_L;         
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//��λ��ָ����� 
		OV7725_RCK_H; 
	
		for(i=0;i<OV7725_WINDOW_HEIGHT;i++)
		{
			for(j=0;j<OV7725_WINDOW_WIDTH;j++)
			{
				OV7725_RCK_L;
				color=GPIOC->IDR&0XFF;	//������
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=GPIOC->IDR&0XFF;	//������
				OV7725_RCK_H; 
				//�ֱ��ȡ�����ֽڵ��������ݲ����뵽16λ�޷�������color��
			  ////////////////////////////////////////////
	              //rgb565 ��ɫͼ��ҶȻ� �ֱ���r g  b ��������
				camera_red=(color&0xF800)>>8;
                   camera_green=(color&0x07E0)>>3;
                   camera_blue=(color&0x001F)<<3;				
				camera_gray=(30*camera_red+59*camera_green+11*camera_blue+50)/100;	
//	���ü�Ȩƽ��ֵ �ҶȻ�		
				////////////////////////////////////////////////
				picture1[(buf_flag% 1024 )] = camera_gray;   
//ÿ��ȡ1024���������ݾʹ��뵽������ȥ
				if(buf_flag%1024==1023)
        {
      	W25QXX_Write(picture1,(address_flag+1)*76800+((buf_flag/1024) * 1024),1024);			
//��w25q128 ���뱳��ͼ�����ݣ�һ֡ͼ����76800KB�����Դ���ͼ���ʱ��Ҫ�ڻ����ĵ�ַ�ϼ���76800��������ţ������ȡ��
		}			
		buf_flag ++;
		LCD->LCD_RAM=color;  
			}
		}
 		ov_sta=0;					//����֡�жϱ��
		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽�� 
	} 
}

//�˺������ڼ�����һҶȻ����ʵʱͼ��ͱ����Ҷ�ͼ�����֡���������֮�����Ĵ�С
//�����趨��ֵ��ͼ��ֳ�0 ��1�����أ�����֮�����̬ѧ���������лҶ�ͼ���ֵ��

void cf(void)
{
	u16 picture1[1024]={0};   // ���ڴ�flash�ж�д����ʱ�������
	u8 picture2[1024]={0};//���ڴ�Ϊ5ȥ28�ж�д����ʱ�������
	u32 i,j;
     for(i=0;i<75;i++)     //һ֡320��240��ͼ�� ��Ҫ75KB�� һ�ζ�ȡ1KB��������Ҫѭ��75��
		{
		STMFLASH_Read(FLASH_SAVE_ADDR+(i * 2048),picture1,1024);  
//FLASH_ADDR Ϊflash��ȡ�Ļ�׼��ַ����Ϊһ������ռ�����ֽڣ�������Ҫ����2048
		W25QXX_Read(picture2,i*1024,1024);//��w25q128��ȡ����ͼ��
		for(j=0;j<1024;j++)         //��ÿһ��������ÿ�����ؽ��в�ֱȽ�
		{
		if(picture1[j]>picture2[j])  
//��Ϊ��֪���Ҷ�ֵ�ĸ������Զ�������ֵҪ�Ƚ��бȽϣ��ô�ļ�ȥС��
		{
	      picture1[j] = picture1[j]-picture2[j];
		if (picture2[j] <30) picture1[j]=0;   
//�������֮��ĻҶȲ���ڵ���30�������ô�����ֵΪ1���������ô�����ֵΪ0
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


//remo_noise������������ȥ����ֵ�����ͼ���е���������Щ����һ�㶼�Ƚ�С�������Ƚ��и�ʴ����
//�������Ͳ���
//ʹ��4����ͬ�����ӣ��ֱ�ȥ����ͬ�����ϵ�����
void remo_noise(void)
{ 
u32 i,j,k;
u16 picture1[960]={0};
u16 picture2[320]={0};
  u16 picture3[320]={0};
///////////////////////////////
//�Ƚ�����ֱ�����ϵĸ�ʴ�����Ͳ�����������������
	for (i=0;i<238;i++)  //��ʴ
{
STMFLASH_Read(FLASH_SAVE_ADDR+i*320,picture1,960);
//������ѭ����ȡ��һ�ζ�ȡ����������960������ �ܹ���Ҫ��ȡ238�Σ���һ�к����һ�к���
	for(k=0;k<320;k++)
	{
	 if(picture1[320+k]==1 &&picture1[k]==1  && picture1[640+k]==1)picture2[k]=1;			
	//��ʴ����		
	else picture2[k]=0;
	}			
STMFLASH_Write(FLASH_SAVE_ADDR2+(i+1)*320,picture2,320);	 
//ѭ������flash�У���Ϊֻ���м�һ�����ã�����ֻ��Ҫ����320������
}
//
for (i=1;i<239;i++)  //���ͣ�ͬ��ʴ������ֻ�������������
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
//����ˮƽ�����ϵĸ�ʴ�����Ͳ�����

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
//��������45�ȷ����ϵĸ�ʴ�����Ͳ�����
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
//��������45�ȷ����ϵĸ�ʴ�����Ͳ�����
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


// r1eg_person()�������ڶ���ͼ���еĸ����������Ƿ����˴���
//
void recg_person(void)
{
u32 i,j,k,m;
	u16 picture1[1920]={0};//ѭ����ȡ��һ�ζ�ȡ6�У��ܹ�1920������
	u32 coment1=0;   //���ڼ���6��������ÿ��6��10��Сͼ�����1�ĸ���
	u32 coment2=0;	
	u32 coment3=0;	
	u32 coment4=0;	
	u32 coment5=0;	
	u32 coment6=0;
	u8 flag1=0;   //���ڱ��6���������Ƿ�����
	u8 flag2=0;
	u8 flag3=0;
	u8 flag4=0;
	u8 flag5=0;
	u8 flag6=0;	
////////////////////////////
//����1������2 ��	��Աʶ��
  for(i=0;i<16;i++)    // ��������һ������2��ͼ������ռ�����Լռ50�У�ÿ����Ҫ��ȡ6��10��С��ͼ��飬���Ҽ����������ȡ������Ҫѭ��16��
	{	
		STMFLASH_Read(FLASH_SAVE_ADDR+(i * 960),picture1,1920);
//һ�ζ�ȡ6�У�������Ҫ��ȡ1920������
	  for (j=0;j<31;j++)
		{  
		  for(k=0;k<6;k++)
			{
				for(m=0;m<10;m++)
			   {
				  if(picture1[k*320+m+j*5]==1) coment1++;   
 //����ÿһ��6��10��С�ķ���������Ϊ1�ĸ�����
				  } 
			}
		if(coment1>55) flag1=1;   
//��������������55������Ϊ���������ˣ���flag1����1 ��������ѭ��
		if(flag1==1)  break;
		}
		for (j=32;j<63;j++)   //�������2����ͬ���Ĳ���
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
	for(i=0;i<13;i++)   //��������������Լռ40�У�������Ҫѭ��13�� ����ԭ��ͬ��
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
for(i=0;i<10;i++)//����5������6Լռ30�У�������Ҫѭ��10�� ����ԭ��ͬ��
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
	//���ֲ��������ݸ�ȫ�ֱ���
}
//����LCD��ʾ(OV7725ͼ�񴫸�����ϵͳʹ�ô˴�����)
//���Ҵ˹��̽���ȡ����ͼ����Ϣֱ�ӽ��лҶȻ��洢��flash��
void OV7725_camera_refresh(void)
{
	u32 i,j;
	u16 picture2[1024]={0};    //�м�洢ͼ����������
 	u16 color;                //��ov7725��ȡ�Ĳ�ɫ���ش������
    u16 camera_temp;
    u8 camera_red;     //R G B ��������  �Լ��Ҷ����ط���
    u8 camera_green;
    u8 camera_blue;	
	u8 camera_gray;
	//////////////change
	u32 brightness1 = 0;    //����ͼ��������50��50��С�ĻҶ�ͼ��ƽ��ֵ
	u32 buf_flag = 0;      //�Ҷ�ֵ�Ƿ������ֵ�ı�ʶ��1��ʶ���ڣ�0��ʶ������
	u8 t=0;	
	u8 buf2[1]={0xFF};  
	///////////////change
	if(ov_sta)//ov7725��֡�жϸ���
	{
		LCD_Scan_Dir(U2D_L2R);		//���ϵ���,������
	LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT);//����ʾ�������õ���Ļ����
		if(lcddev.id==0X1963)
	LCD_Set_Window((lcddev.width-OV7725_WINDOW_WIDTH)/2,(lcddev.height-OV7725_WINDOW_HEIGHT)/2,OV7725_WINDOW_HEIGHT,OV7725_WINDOW_WIDTH);//����ʾ�������õ���Ļ����
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	
		OV7725_RRST=0;				//��ʼ��λ��ָ�� 
		OV7725_RCK_L;
		OV7725_RCK_H;
		OV7725_RCK_L;
		OV7725_RRST=1;				//��λ��ָ����� 
		OV7725_RCK_H; 
		for(i=0;i<OV7725_WINDOW_HEIGHT;i++)   //�ܹ���240��
		{
			for(j=0;j<OV7725_WINDOW_WIDTH;j++)  //ÿ����320������
			{
				OV7725_RCK_L;
				color=GPIOC->IDR&0XFF;	//������
				OV7725_RCK_H; 
				color<<=8;  
				OV7725_RCK_L;
				color|=GPIOC->IDR&0XFF;	//����ɫͼ�����ݣ���ʶ��color��
				OV7725_RCK_H; 
			  ////////////////////////////////////////////
	camera_red=(color&0xF800)>>8;       //��ȡ������������Ҷ�ֵ
     camera_green=(color&0x07E0)>>3;
     camera_blue=(color&0x001F)<<3;		
	camera_gray=(30*camera_red+59*camera_green+11*camera_blue+50)/100;  
//��������50��50��С�ĻҶ�ƽ��ֵ���д�160��209���д�40��90,20��299
if((i>=160 && i<210 && j>=40 &&j<90)||(i>=160 && i<210 && j>=250 &&j<300))
	{
	      brightness1 =brightness1 + camera_gray;
				}
	////////////////////////////////////////////////
	//	���Ҷ���������ÿ��1024���ʹ��뵽flash��
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
LCD->LCD_RAM=color;  //��ͼ��������ʾ��LCD��
}
}
brightness1 = brightness1/5000;   //����Ҷ�ƽ��ֵ
if(brightness1 < 120)b_flag=1;    //��ʵʱͼ��ĻҶ�ƽ��ֵ�Ƿ������ֵ��������ֵ������ǿ�������ƣ�b_flag=0����֮����������Ҫ���ƣ�b_flag=1
else b_flag=0;
 	ov_sta=0;					//����֡�жϱ��
		ov_frame++; 
		LCD_Scan_Dir(DFT_SCAN_DIR);	//�ָ�Ĭ��ɨ�跽�� 
	} 
}
//////////////////////
//���ڵ�ż̵�����Ҫ6���룬������Ҫ��PD11��PD12 PD13 PE0 PE1 PE2 �Ĵ��ڽ��г�ʼ����֮����ܷ������
void ele_mag_Init(void)
{
GPIO_InitTypeDef  GPIO_InitStructure;	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);	 
//ʹ��PD,PE�˿�ʱ��
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;	 //PD�˿ڵĹܽ�Ϊ11 12 13
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOD, &GPIO_InitStructure);	
////////////////////////////
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;	//PE�˿ڵĹܽ�Ϊ0 1 2
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOE, &GPIO_InitStructure);		//�����趨������ʼ��GPIOB.5
  GPIO_ResetBits(GPIOD,GPIO_Pin_11);
GPIO_ResetBits(GPIOD,GPIO_Pin_12);
GPIO_ResetBits(GPIOD,GPIO_Pin_13);
  GPIO_ResetBits(GPIOE,GPIO_Pin_0);    //����6������˿ڶ�����Ϊ0�����ر����еƹ�
GPIO_ResetBits(GPIOE,GPIO_Pin_1);
GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	
}
//�˺����������ǽ�6����ʾ�������Ƿ����˵ı�ʶ��һ���ֽڱ�ʾ��������������һ����ʾ��ÿһλ�ֱ��ʾ�������Ƿ�����
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

//�˺������ڽ����ұ���ģ����뵽w25q128оƬ��ȥ������6��ѭ��
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
			//���ȶ�6������ڽ������ã����Ƶƹ������   
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
			delay_ms(1000);   //�ӳ�1�룬��֤ͼ���ȶ�
			bk_refresh(date_flag) ;
				
			}
			}
			}
			}
		}	
	}
}
int main(void)   //������
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
	u8 msgbuf[15];//��Ϣ������
	u8 tm=0;
	u8 lightmode=0,effect=0;
	s8 saturation=0,brightness=0,contrast=0;

	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ 115200
 	usmart_dev.init(72);		//��ʼ��USMART		
 	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD  
	W25QXX_Init();			//W25QXX��ʼ��
	TPAD_Init(6);				//����������ʼ�� 
	ele_mag_Init();     //��ż̵�������˿ڳ�ʼ��
 	POINT_COLOR=RED;			//��������Ϊ��ɫ 
	LCD_ShowString(30,50,200,16,16,"Classroom lighting control");	
	LCD_ShowString(30,70,200,16,16,"OV7725 TEST");	
	LCD_ShowString(30,130,200,16,16,"    ");
	LCD_ShowString(30,150,200,16,16,"    ");
	LCD_ShowString(30,170,200,16,16,"    ");
	LCD_ShowString(30,190,200,16,16,"    ");	 
  LCD_ShowString(30,210,200,16,16,"OV7725 Init...");	 	
		
while(W25QXX_ReadID()!=W25Q128)			//��ⲻ��W25Q128
	{
		LCD_ShowString(30,270,200,16,16,"W25Q128 Check Failed!");
		delay_ms(500);
		LCD_ShowString(30,290,200,16,16,"Please Check!        ");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}
	LCD_ShowString(30,290,200,16,16,"W25Q128 Ready!");    
	/////////////////////change
	while(1)//��ʼ��OV7725
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
					 OV7725_Window_Set(OV7725_WINDOW_WIDTH,OV7725_WINDOW_HEIGHT,0);//QVGAģʽ���
			    OV7725_Light_Mode(lightmode);    //����ov7725����
			    OV7725_Color_Saturation(saturation);
			    OV7725_Brightness(brightness);
			    OV7725_Contrast(contrast);
			    OV7725_Special_Effects(effect);
			    OV7725_CS=0;
			    TIM6_Int_Init(10000,7199);	//10Khz����Ƶ��,1�����ж�					  
	        EXTI8_Init();						//ʹ���ⲿ�ж�8,����֡�ж�			
	        LCD_Clear(BLACK);	//�����Ļ
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
				TIM6_Int_Init(10000,7199);			//10Khz����Ƶ��,1�����ж�			
	              EXTI8_Init();						//ʹ���ⲿ�ж�8,����֡�ж�		
	              LCD_Clear(BLACK);	//�����Ļ
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
			OV7670_Window_Set(12,176,240,320);//���ô���
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
	OV7725_camera_refresh();  //���Ȼ�ȡһ֡ͼ�񣬼����ض�����ĻҶ�ƽ��ֵ
	if(b_flag==1)   //b_flag����1˵����Ҫ����
	{
	GPIO_SetBits(GPIOD,GPIO_Pin_11);  //6��ƹ�ȫ����,GPIO����ߵ�ƽ1
	GPIO_SetBits(GPIOD,GPIO_Pin_12);
	GPIO_SetBits(GPIOD,GPIO_Pin_13);
     GPIO_SetBits(GPIOE,GPIO_Pin_0);
	GPIO_SetBits(GPIOE,GPIO_Pin_1);
	GPIO_SetBits(GPIOE,GPIO_Pin_2);
	for (x_flag=0;x_flag<60;x_flag++)
		{
		delay_ms(1000);//�ӳ� 60s  ����ѧ��ѡ����λ
		}
for(j=0;j<75;j++)   
//���±����Ҷ�ͼ�񣬽����ҵƹ�ȫ��ʱ�ĻҶ�ͼ��洢��w25q128�ʼ��λ�ã�������в��
{
W25QXX_Read(picture4,64*76800+j*1024,1024);
W25QXX_Write(picture4,j * 1024,1024);		
}
		
	}
	else
	{
	for(j=0;j<75;j++)   //���±����Ҷ�ͼ�񣬽����ҵ�ȫ��ʱ��Ҷ�ͼ��洢��w25q128�ʼ��λ�ã�������в��
	{
	W25QXX_Read(picture4,76800+j*1024,1024);
	W25QXX_Write(picture4,j * 1024,1024);		
	}
	}
	///////////////////////
 	while(1)
	{	
		
		if(TPAD_Scan(0))//��⵽�������� 
		{	
		  update_bk();  //���±���ͼ��
		}
		if(sensor==OV7725)OV7725_camera_refresh();		//ov7725������ʾ����¼ƽ���Ҷ�ֵ
//		else if(sensor==OV7670)OV7670_camera_refresh();	//������ʾ
 		b_buff<<=1;   //����һλ�����λ���0�����λ���������λ���ڼ�¼���µĻҶ�ƽ��ֵ��
		if (b_flag==1)b_buff+=1;  
		if(b_buff==0XFF)	  
 //���b_buffΪ0XFF��˵������8֡ͼ��ĻҶ�ƽ��ֵ��������ֵ��������Ҫ����
		{
		cf();     //�Ҷ�ͼ�񱳾���֣������ж�ֵ��
   		remo_noise();  //ͼ��ȥ�룬��ֵ����̬ѧ����
		recg_person();		//����������Ƿ�����
	     updatebuff2=six2one(flag1_w,flag2_w,flag3_w,flag1_w,flag4_w,flag5_w);  
//�����Ӧ����ͼ��洢���׵�ַ
		if(updatebuff2!=updatebuff1)   //���updatebuff2����updatebuff1����˵������Ҫ���±���ͼ�񣬷�����Ҫ���±���ͼ��
			{
			if(flag1_w==1)GPIO_SetBits(GPIOD,GPIO_Pin_11);  //���Ľ��ҵƹ�
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
//���±����Ҷ�ͼ�񣬽��Ҷȱ���ͼ��洢��w25q128�ʼ��λ�ã�������в��
				{
				  W25QXX_Read(picture4,(updatebuff2+1)*76800+j*1024,1024);
				  W25QXX_Write(picture4,j * 1024,1024);		
				}
			}		
		}
		i++;
		if(i>=15)//DS0��˸.
		{
			i=0;
			LED0=!LED0;
 		}
	}	   
}