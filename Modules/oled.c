

#include "oled.h"
#include "stdlib.h"	 
#include "spi.h"

void LCD_WR_DATA8(char da) //发送数据-8位参数
{	
  OLED_CS_Clr();
  OLED_DC_Set();
	//LCD_Writ_Bus(da);  
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&da, 1, 0xFFFF);
	OLED_CS_Set();
}  
 void LCD_WR_DATA(int da)
{	
  uint8_t buff[2];
  OLED_CS_Clr();
  OLED_DC_Set();
	//LCD_Writ_Bus(da>>8);
  //LCD_Writ_Bus(da);
  //HAL_SPI_Transmit(&hspi1, (uint8_t*)&da, 2, 0xFFFF);
  buff[0] = (uint8_t)(da>>8);
  buff[1] = (uint8_t)(da&0Xff);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&buff[0], 2, 0xFFFF);
  //HAL_SPI_Transmit(&hspi1, (uint8_t*)&buff[1], 1, 0xFFFF);
	OLED_CS_Set();
}	  

uint8_t lcdDataFalg = 0;
void LCD_WR_DATAS(uint16_t* da, uint16_t len)
{	
  OLED_CS_Clr();
  OLED_DC_Set();
  hspi1.Instance->CR1 |= (0x1<<11);
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  #if 1
  while(lcdDataFalg);
  lcdDataFalg = 1;
  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)da, len);
  while(lcdDataFalg);
  #else
  HAL_SPI_Transmit(&hspi1, (uint8_t*)da, len, 0xFFFF);
  hspi1.Instance->CR1 &= ~(0x1<<11);
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  OLED_CS_Set();
  #endif
}	
void LCD_WR_REG(char da)	 
{		
  while(lcdDataFalg);
  OLED_CS_Clr();
  OLED_DC_Clr();
	//LCD_Writ_Bus(da);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&da, 1, 0xFFFF);
	OLED_CS_Set();
}
 void LCD_WR_REG_DATA(int reg,int da)
{
  LCD_WR_REG(reg);
	LCD_WR_DATA(da);
}

void Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{ 
   LCD_WR_REG(0x2a);
   LCD_WR_DATA8(x1>>8);
   LCD_WR_DATA8(x1);
   LCD_WR_DATA8(x2>>8);
   LCD_WR_DATA8(x2);
  
   LCD_WR_REG(0x2b);
   LCD_WR_DATA8(y1>>8);
   LCD_WR_DATA8(y1);
   LCD_WR_DATA8(y2>>8);
   LCD_WR_DATA8(y2);

   LCD_WR_REG(0x2C);					 						 
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi == &hspi1)
  {
    hspi1.Instance->CR1 &= ~(0x1<<11);
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    OLED_CS_Set();
    lcdDataFalg = 0;
  }
}

void Lcd_Init(void)
{
	OLED_BLK_Clr();
	OLED_RST_Clr();
	HAL_Delay(250);
	OLED_RST_Set();
	HAL_Delay(250);
									
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36); 
	LCD_WR_DATA8(0x74);

	LCD_WR_REG(0x3A); 
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33);

	LCD_WR_REG(0xB7); 
	LCD_WR_DATA8(0x35);  

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x19);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x12);   

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20);  

	LCD_WR_REG(0xC6); 
	LCD_WR_DATA8(0x0F);    

	LCD_WR_REG(0xD0); 
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2B);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4C);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x51);
	LCD_WR_DATA8(0x2F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0x21); 

	LCD_WR_REG(0x11); 
	//Delay (120); 

	LCD_WR_REG(0x29); 

	LCD_Clear(WHITE);
	
} 

//清屏函数
//Color:要清屏的填充色
void LCD_Clear(uint16_t Color)
{
	uint16_t i,j;  	
	Address_set(0,0,LCD_W-1,LCD_H-1);
    for(i=0;i<LCD_W;i++)
	 {
	  for (j=0;j<LCD_H;j++)
	   	{
        	LCD_WR_DATA(Color);	 			 
	    }

	  }
}





































