#ifndef __MALLOC_H
#define __MALLOC_H

#include <main.h>


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
	  
#ifndef NULL
#define NULL 0
#endif

#define SRAMIN1	 1	//�ڲ��ڴ��1
#define SRAMIN2  0	//�ڲ��ڴ��2

//mem1�ڴ�����趨.mem1��ȫ�����ڲ�SRAM1����
#define MEM1_BLOCK_SIZE			32  	  						//�ڴ���СΪ32�ֽ�
#define MEM1_MAX_SIZE			1*1024  						//�������ڴ� 116K
#define MEM1_ALLOC_TABLE_SIZE	MEM1_MAX_SIZE/MEM1_BLOCK_SIZE 	//�ڴ����С

//mem2�ڴ�����趨.mem2���ڴ�ش����ڲ�SRAM2����
#define MEM2_BLOCK_SIZE			32  	  						//�ڴ���СΪ32�ֽ�
#define MEM2_MAX_SIZE			60*1024  						//�������ڴ�60K
#define MEM2_ALLOC_TABLE_SIZE	MEM2_MAX_SIZE/MEM2_BLOCK_SIZE 	//�ڴ����С
		 	 
//�ڴ����������
struct _m_mallco_dev
{
	void (*init)(u8);					//��ʼ��
	u8 (*perused)(u8);		  	    	//�ڴ�ʹ����
	u8 	*membase[2];					//�ڴ�� ����3��������ڴ�
	u16 *memmap[2]; 					//�ڴ����״̬��
	u8  memrdy[2]; 						//�ڴ�����Ƿ����
};
extern struct _m_mallco_dev mallco_dev;	 //��mallco.c���涨��

void mymemset(void *s,u8 c,u32 count);	 //�����ڴ�
void mymemcpy(void *des,void *src,u32 n);//�����ڴ�     
void mem_init(u8 memx);					 //�ڴ������ʼ������(��/�ڲ�����)
u32 mem_malloc(u8 memx,u32 size);		 //�ڴ����(�ڲ�����)
u8 mem_free(u8 memx,u32 offset);		 //�ڴ��ͷ�(�ڲ�����)
u8 mem_perused(u8 memx);				 //����ڴ�ʹ����(��/�ڲ�����) 
////////////////////////////////////////////////////////////////////////////////
//�û����ú���
void myfree(u8 memx,void *ptr);  			//�ڴ��ͷ�(�ⲿ����)
void *mymalloc(u8 memx,u32 size);			//�ڴ����(�ⲿ����)
void *myrealloc(u8 memx,void *ptr,u32 size);//���·����ڴ�(�ⲿ����)


#endif












