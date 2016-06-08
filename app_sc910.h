#ifndef __APP_SC910_H__
#define __APP_SC910_H__
#include "adc_hw_trigger.h"
#include "board.h"

#define AUTO_MODE       0
#define MANUAL_MODE     0x55aa
#define REGULATE_MODE   0x6273

#define SAVE_PARAM      1
#define SAVE_COMPLETED  0
#define SAVE_OVERTIMES  10
#define SAVE_PARAM_ADDR 0xF800
#define SAVE_TIMES_MAX  10

#define DA_MAX_VALUE    4095
#define PWM_MAX_VALUE   65535
#define AD_MAX_VALUE    4095

#define GIVE_MAX_LOGIC  30000
#define GIVE_MIN_LOGIC  -30000

//FOR AO LOW Bit �����������ѡ��High Bit��ʾ�����������LVDT������� 
//FOR AI LOB BIT ���������HIGH BIT������
//AI1 ��ӦAI1, AI2��ӦAI2�����ǵ�ѹ���룬0~2.5V��AI3��LVDT�ķ������룬0~2.5V
//AO1 ��ӦAO1, �����������ʹ���ڲ�K02оƬDAC���������ڻ�׼�ӵ�2.5V������ֻ�����2.5V��ѹ
//AO2 ��ӦAO2, �������������ʹ��PWMģ��
//AO3 ��ӦLVDT�����������������ʹ��PWMģ��

#define LOW_BIT0      (1 << 0) 
#define LOW_BIT1      (1 << 1)
#define LOW_BIT2      (1 << 2)
#define LOW_BIT3      (1 << 3)

#define HIGH_BIT0      (1 << 4)
#define HIGH_BIT1      (1 << 5)
#define HIGH_BIT2      (1 << 6)
#define HIGH_BIT3      (1 << 7)

#define AO_CHSEL_NONE                                   0x00
#define AO_CHSEL_AO1_DRIVE_AO2_FEEDBACK                 (HIGH_BIT1 | LOW_BIT0)
#define AO_CHSEL_AO1_DRIVE_AO3_FEEDBACK                 (HIGH_BIT2 | LOW_BIT0)
#define AO_CHSEL_AO2_DRIVE_AO1_FEEDBACK                 (HIGH_BIT0 | LOW_BIT1)
#define AO_CHSEL_AO2_DRIVE_AO3_FEEDBACK                 (HIGH_BIT2 | LOW_BIT1)
#define AO_CHSEL_AO3_DRIVE_AO1_FEEDBACK                 (HIGH_BIT0 | LOW_BIT2)
#define AO_CHSEL_AO3_DRIVE_AO2_FEEDBACK                 (HIGH_BIT1 | LOW_BIT2)

#define AO_CHSEL_AO1_DRIVE_LVDT_FEEDBACK                (HIGH_BIT3 | HIGH_BIT1| LOW_BIT0)
#define AO_CHSEL_AO2_DRIVE_LVDT_FEEDBACK                (HIGH_BIT3 | HIGH_BIT0| LOW_BIT1)

#define AI_CHSEL_NONE                                   0x00    
#define AI_CHSEL_AI1_GIVEN_AI2_FEEDBACK                 (HIGH_BIT1 | LOW_BIT0)  
#define AI_CHSEL_AI1_GIVEN_AI3_FEEDBACK                 (HIGH_BIT2 | LOW_BIT0) 
#define AI_CHSEL_AI2_GIVEN_AI1_FEEDBACK                 (HIGH_BIT0 | LOW_BIT1) 
#define AI_CHSEL_AI2_GIVEN_AI3_FEEDBACK                 (HIGH_BIT2 | LOW_BIT1) 
#define AI_CHSEL_AI3_GIVEN_AI1_FEEDBACK                 (HIGH_BIT0 | LOW_BIT2)
#define AI_CHSEL_AI3_GIVEN_AI2_FEEDBACK                 (HIGH_BIT1 | LOW_BIT2)  

#define AI_CHSEL_AI1_GIVEN_LVDT_FEEDBACK                (HIGH_BIT3 | LOW_BIT0)
#define AI_CHSEL_AI2_GIVEN_LVDT_FEEDBACK                (HIGH_BIT3 | LOW_BIT1)


typedef struct tagParameter
{ 
  int16_t               g_ucRunMode;            //����ģʽ
  
  int16_t               AI_CHSEL;               //AIͨ��ѡ��
  //��ֵʹ���߼�ֵ
  int16_t	        AI1_MAX_VALUE;          //AI1ͨ����ֵ
  int16_t	        AI1_MIN_VALUE;          //AI1ͨ����ֵ
  int16_t	        AI2_MAX_VALUE;          //AI2ͨ����ֵ
  int16_t	        AI2_MIN_VALUE;          //AI2ͨ����ֵ
  int16_t	        AI3_MAX_VALUE;          //AI3ͨ����ֵ
  int16_t	        AI3_MIN_VALUE;          //AI3ͨ����ֵ
  
  int16_t               LVDT_MID_VALUE;         //LVDT��λֵ
  
  int16_t               AO_CHSEL;               //AOͨ��ѡ��
  int16_t	        AO1_MAX_VALUE;
  int16_t	        AO1_MIN_VALUE;
  int16_t	        AO2_MAX_VALUE;
  int16_t	        AO2_MIN_VALUE;
  int16_t	        AO3_MAX_VALUE;
  int16_t	        AO3_MIN_VALUE;
  
  
  int16_t               PID_K;                  //PID����
  int16_t               OFFSET;                  //ƫ��
  
}SC910Parameter;

typedef struct tagInputData
{
  int16_t       DI_DATA_VALUE;
  
  int16_t       AI1_DATA_VALUE;
  int16_t       AI2_DATA_VALUE;
  int16_t       AI3_DATA_VALUE;    
  int16_t       AI_LVDT_DATA_VALUE;  
}
SC910InputData;

typedef struct tagOutputData
{
  int16_t     DO_DATA_VALUE;
  
  int16_t    AO1_DATA_VALUE;
  int16_t    AO2_DATA_VALUE;
  int16_t    AO3_DATA_VALUE;    
}
SC910OutputData;

typedef struct tagCommData
{
  int16_t       DI_DATA_VALUE;
  int16_t       DO_DATA_VALUE;
  
  int16_t       SAVE_CMD;
  int16_t       RUN_MODE;
  int16_t       AI_CHSEL;               //AIͨ��ѡ��
  int16_t	AI1_MAX_VALUE;
  int16_t	AI1_MIN_VALUE;
  int16_t	AI2_MAX_VALUE;
  int16_t	AI2_MIN_VALUE;
  int16_t	AI3_MAX_VALUE;
  int16_t	AI3_MIN_VALUE;
  
  int16_t       AO_CHSEL;               //AOͨ��ѡ��
  int16_t	AO1_MAX_VALUE;
  int16_t	AO1_MIN_VALUE;
  int16_t	AO2_MAX_VALUE;
  int16_t	AO2_MIN_VALUE;
  int16_t	AO3_MAX_VALUE;
  int16_t	AO3_MIN_VALUE;
  
  int16_t       PID_K;                  //PID����
  int16_t       OFFSET;                  //ƫ��
  
  int16_t       AI1_DATA_VALUE;
  int16_t       AI2_DATA_VALUE;
  int16_t       AI3_DATA_VALUE;
  int16_t       AI_LVDT_DATA_VALUE;
  
  int16_t       AO1_DATA_VALUE;
  int16_t       AO2_DATA_VALUE;
  int16_t       AO3_DATA_VALUE;  
  
}
SC910CommData;



extern void sc910_input();
extern void sc910_iec();
extern void sc910_output();
extern void sc910_regulate(); 
extern void sc910_load_param();
extern void sc910_comm_process();

void sc910_input_convert(SC910InputData *raw, SC910InputData *logic);
void sc910_output_convert(SC910OutputData *raw, SC910OutputData *logic);
void sc910_limit_value(int16_t *dst, int16_t high, int16_t low);
void sc910_force_value(int16_t *dst, int16_t high, int16_t low, int16_t new_value,int16_t retain);
void sc910_regulate_write();
void sc910_write_reg(int16_t *param, int16_t *commdata, int16_t new_value, int16_t old_value);

void seek_max_min(uint16_t * max, uint16_t * min, uint16_t * max_pos, uint16_t * min_pos, uint16_t *pbuf, uint16_t len);
void sc910_save_param();

#endif