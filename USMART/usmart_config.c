#include "usmart.h"
#include "usmart_str.h"
////////////////////////////用户配置区///////////////////////////////////////////////
//这下面要包含所用到的函数所申明的头文件(用户自己添加) 
#include "delay.h"	 	
#include "sys.h"
//#include "lcd.h"
#include "StepMotor/bsp_STEPMOTOR.h"

//      STEPMOTOR_DisMoveRel(-100,step_accel,step_decel,set_speed);//向前移动100mm
#define  FASTSEEK_SPEED   300		//原点回归速度
#define  SLOWSEEK_SPEED   100		//原点回归爬行速度
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
// 速度最大值由驱动器和电机决定，有些最大是1800，有些可以达到4000
//uint32_t set_speed  = 500;         // 速度 单位为0.05rad/sec
//// 加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
//// 所以加速度和减速度值一般是在实际应用中多尝试出来的结果
//uint32_t step_accel = 150;         // 加速度 单位为0.025rad/sec^2
//uint32_t step_decel = 50;         // 减速度 单位为0.025rad/sec^2
extern void STEPMOTOR_DisMoveAbs(__IO uint16_t Target_Dis, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed);

extern void STEPMOTOR_DisMoveRel(__IO int16_t distance, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed);
extern uint8_t search_flag ;

void beginsearch(int8_t dir)								 
{
	HomeDir = dir;
	search_flag = 1;
//	while(HomeCapture == FALSE)
//	{
//		STEPMOTOR_AxisHome(FASTSEEK_SPEED,SLOWSEEK_SPEED,150,50);	//搜索原点
//	}
}
//extern void led_set(u8 sta);
//extern void test_fun(void(*ledset)(u8),u8 sta);										  
//函数名列表初始化(用户自己添加)
//用户直接在这里输入要执行的函数名及其查找串
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//如果使能了读写操作
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",	 
#endif		   
	(void*)delay_ms,"void delay_ms(u16 nms)",
 	(void*)delay_us,"void delay_us(u32 nus)",	 
//	(void*)LCD_Clear,"void LCD_Clear(u16 Color)",
//	(void*)LCD_Fill,"void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)",
//	(void*)LCD_DrawLine,"void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)",
//	(void*)LCD_DrawRectangle,"void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)",
//	(void*)LCD_Draw_Circle,"void Draw_Circle(u16 x0,u16 y0,u8 r)",
//	(void*)LCD_ShowNum,"void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)",
//	(void*)LCD_ShowString,"void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p)",
//	(void*)LCD_Fast_DrawPoint,"void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)",
//	(void*)LCD_ReadPoint,"u16 LCD_ReadPoint(u16 x,u16 y)",							 
//	(void*)LCD_Display_Dir,"void LCD_Display_Dir(u8 dir)",
//	(void*)LCD_ShowxNum,"void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)", 
//		
//	(void*)led_set,"void led_set(u8 sta)",
//	(void*)test_fun,"void test_fun(void(*ledset)(u8),u8 sta)",						
	
	  (void*)STEPMOTOR_DisMoveRel,"void STEPMOTOR_DisMoveRel(__IO int16_t distance, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)",
		(void*)beginsearch,"beginsearch(int8_t dir)",	
		(void*)STEPMOTOR_DisMoveAbs," STEPMOTOR_DisMoveAbs(__IO uint16_t Target_Dis, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)",

};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//函数控制管理器初始化
//得到各个受控函数的名字
//得到函数总数量
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//函数数量
	0,	  	//参数数量
	0,	 	//函数ID
	1,		//参数显示类型,0,10进制;1,16进制
	0,		//参数类型.bitx:,0,数字;1,字符串	    
	0,	  	//每个参数的长度暂存表,需要MAX_PARM个0初始化
	0,		//函数的参数,需要PARM_LEN个0初始化
};   



















