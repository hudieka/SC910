
// Standard C Included Files
#include <string.h>
#include <stdio.h>
#include "app_sc910.h"

//#define PWM_LVDT_TEST
//#define PWM_AO_TEST
//#define DA_TEST

//uint16_t g_ucRunMode = 0;                        //0表示自动模式，1表示手动模式，2表示整定模式

SC910Parameter g_sc910param;                    //参数
SC910InputData g_sc910inputdata;                //实时输入数据
SC910OutputData g_sc910outputdata;               //实时输出数据
SC910CommData *g_psc910commdata;                //通信缓冲区指针
SC910CommData g_sc910commdata;                  //通信数据备份

SC910InputData g_sc910inputlogicdata;                //实时输入数据
SC910OutputData g_sc910outputlogicdata;               //实时输出数据


//以下参数为逻辑参数
// -30000表示全关，30000表示全开


int g_iLogic_given = 0;                               //给定：范围-30000 +30000
int g_iLogic_offset = 0;                              //偏置:
int g_iLogic_feedback = 0;                            //反馈
long g_iLogic_realoutput = 0;                          //PID实际输出
float g_iLogic_Kp = 0;                                  //比例系数
//逻辑参数end

//以下数据为通信变量
int g_iCom_savepara = 0;

extern unsigned char g_ucMbuf[];
extern uint16_t g_uiAI3[];
extern uint16_t g_uiAI1[];
extern uint16_t g_uiAI2[];

extern uint16_t g_uiAI1_Current;
extern uint16_t g_uiAI2_Current;
extern uint16_t g_uiAI3_Current;

extern uint16_t g_uilvdtbuf[NR_SAMPLES];
extern unsigned char g_uilvdtflag;                       //一个LVDT周期数据数据采集完成，0表示未完成，1表示完成



void sc910_load_param()
{
  g_sc910param = *((SC910Parameter *)SAVE_PARAM_ADDR);
  g_psc910commdata = (SC910CommData *)g_ucMbuf;
  g_sc910commdata = *g_psc910commdata;

#if 0  
  //yangliang for test
  g_sc910param.AO_CHSEL = AO_CHSEL_AO1_DRIVE_LVDT_FEEDBACK;
  g_sc910param.AI1_MAX_VALUE = GIVE_MAX_LOGIC;
  g_sc910param.AI1_MIN_VALUE = GIVE_MIN_LOGIC;
  g_sc910param.AI2_MAX_VALUE = GIVE_MAX_LOGIC;
  g_sc910param.AI2_MIN_VALUE = GIVE_MIN_LOGIC;
  g_sc910param.AI3_MAX_VALUE = GIVE_MAX_LOGIC;
  g_sc910param.AI3_MIN_VALUE = GIVE_MIN_LOGIC;
  
  g_sc910param.AO1_MAX_VALUE = GIVE_MAX_LOGIC;
  g_sc910param.AO1_MIN_VALUE = GIVE_MIN_LOGIC;
  g_sc910param.AO2_MAX_VALUE = GIVE_MAX_LOGIC;
  g_sc910param.AO2_MIN_VALUE = GIVE_MIN_LOGIC;
  g_sc910param.AO3_MAX_VALUE = GIVE_MAX_LOGIC;
  g_sc910param.AO3_MIN_VALUE = GIVE_MIN_LOGIC;
  
  g_sc910param.AI_CHSEL = AI_CHSEL_AI1_GIVEN_LVDT_FEEDBACK;
  g_sc910param.LVDT_MID_VALUE = 4000;
  g_sc910param.PID_K = 1;
  g_sc910param.OFFSET = 0;
#endif
  
  g_sc910param.g_ucRunMode = AUTO_MODE;
  
  
  //g_lvdtseek.flag = SEEK_PHASE_MODE;
  
#if 0
  if(g_sc910param.AO_CHSEL & (HIGH_BIT3 | HIGH_BIT2| LOW_BIT2))
  {
      //enable_lvdt_pwm();    
  }
  
  if(g_sc910param.AO_CHSEL & (HIGH_BIT1| LOW_BIT1))
  {
      //enable_ao2_pwm();
  }
#endif
  
}

void sc910_save_param()
{
  if(SAVE_PARAM == g_iCom_savepara)
  {
    HSRUN_TO_RUN();
    
    FLASH_EraseSector(SAVE_PARAM_ADDR);   
    FLASH_PROGRAM(SAVE_PARAM_ADDR, (LWord *)(&g_sc910param), sizeof(g_sc910param)/4);
    g_iCom_savepara = SAVE_COMPLETED;
    g_psc910commdata->SAVE_CMD = SAVE_COMPLETED;
    g_sc910commdata.SAVE_CMD = SAVE_COMPLETED;
    
    RUN_TO_HSRUN();
  }
}
    
#define LVDT_AVGNUMBER   4

volatile uint16_t g_uitestbuf[1000];

uint16_t lvdt_avg[LVDT_AVGNUMBER];
uint16_t ai1_avg[LVDT_AVGNUMBER];
uint16_t ai2_avg[LVDT_AVGNUMBER];
uint16_t ai3_avg[LVDT_AVGNUMBER];

volatile uint16_t guicount = 0;

uint16_t sc910_window_filter(uint16_t *buf, uint16_t *p_num, uint16_t size, uint16_t new_value)
{
      uint32_t value = 0;
      uint16_t i = 0 ,  ret = 0;
      uint16_t num = *p_num;
      
      if(num < size)
      {
          buf[num] = new_value;
          
          for(i = 0; i < num; i++)
          {
              value += buf[i];
          }
          
          ret = value / (num+1);
          
          *p_num++;
          
      }
      else
      {
          //Move the old data
          for(i = 0; i < (size - 1); i++)
          {
              buf[i] = buf[i + 1];
              value += buf[i];

          }
          buf[size - 1] = new_value;
          value += buf[size - 1];
          
          ret = value / size;
          
          
      }
      
      return ret;

}

void sc910_input()
{
  static uint16_t lvdt_index = 0, ai1_index = 0, ai2_index = 0, ai3_index = 0;
  
  uint16_t max = 0, min = 0, max_pos = 0, min_pos = 0,lvdt_temp = 0;
  
#if 0
  //Need to change
  g_sc910inputdata.AI1_DATA_VALUE = g_uiAI1_Current;
  
  g_sc910inputdata.AI2_DATA_VALUE = g_uiAI2_Current;
  
  g_sc910inputdata.AI3_DATA_VALUE = g_uiAI3_Current;

  g_sc910inputdata.DI_DATA_VALUE = DI1 |(DI2 <<1) |(DI3 <<2);
#endif  
  

  g_sc910inputdata.AI1_DATA_VALUE = sc910_window_filter(&ai1_avg[0], &ai1_index, LVDT_AVGNUMBER, g_uiAI1_Current);
  g_sc910inputdata.AI2_DATA_VALUE = sc910_window_filter(&ai2_avg[0], &ai2_index, LVDT_AVGNUMBER, g_uiAI2_Current);
  g_sc910inputdata.AI3_DATA_VALUE = sc910_window_filter(&ai3_avg[0], &ai3_index, LVDT_AVGNUMBER, g_uiAI3_Current);
  g_sc910inputdata.DI_DATA_VALUE = DI1 |(DI2 <<1) |(DI3 <<2);
  
  if((g_sc910param.AI_CHSEL & HIGH_BIT3) && (g_uilvdtflag == true))
  {
      seek_max_min(&max,&min,&max_pos,&min_pos,g_uilvdtbuf,NR_SAMPLES);
      //g_sc910inputdata.AI_LVDT_DATA_VALUE = ((max - g_sc910param.LVDT_MID_VALUE) + (g_sc910param.LVDT_MID_VALUE - min)) >> 1;
      //For Test
      
      //g_sc910inputdata.AI_LVDT_DATA_VALUE = (max - min) >> 1;
      lvdt_temp = (max - min) >> 1;
         
      g_sc910inputdata.AI_LVDT_DATA_VALUE = sc910_window_filter(&lvdt_avg[0], &lvdt_index, LVDT_AVGNUMBER, lvdt_temp);
      
#if 0     
      if(avg_index < 4)
      {
          lvdt_avg[avg_index] = lvdt_temp;
      }
      else
      {
          lvdt_avg[3] = lvdt_temp;
      }
      
      
      if(avg_index == 0)
      {
          g_sc910inputdata.AI_LVDT_DATA_VALUE = lvdt_temp;
          avg_index++;
      }
      else if(avg_index == 1)
      {
          g_sc910inputdata.AI_LVDT_DATA_VALUE = (lvdt_avg[0] + lvdt_avg[1]) >> 1;
          avg_index++;
      }
      else if(avg_index == 2)
      {
          g_sc910inputdata.AI_LVDT_DATA_VALUE = (lvdt_avg[0] + lvdt_avg[1] + lvdt_avg[2])/3;
          avg_index++;
      }
      else if(avg_index == 3)
      {
          g_sc910inputdata.AI_LVDT_DATA_VALUE = (lvdt_avg[0] + lvdt_avg[1] + lvdt_avg[2] + lvdt_avg[3]) >> 2;
          lvdt_avg[0] = lvdt_avg[1];
          lvdt_avg[1] = lvdt_avg[2];
          lvdt_avg[2] = lvdt_avg[3];
          avg_index++;
      }
      else
      {
          g_sc910inputdata.AI_LVDT_DATA_VALUE = (lvdt_avg[0] + lvdt_avg[1] + lvdt_avg[2] + lvdt_avg[3]) >> 2;
          lvdt_avg[0] = lvdt_avg[1];
          lvdt_avg[1] = lvdt_avg[2];
          lvdt_avg[2] = lvdt_avg[3];
      }
#endif     

      g_uitestbuf[guicount] = g_sc910inputdata.AI_LVDT_DATA_VALUE;
      guicount++;
      
      //For test
      
      //DAC_DRV_Output(BOARD_DAC_DEMO_DAC_INSTANCE, g_sc910inputdata.AI_LVDT_DATA_VALUE >> 4);
      
      if(guicount >= 1000)
      {
          guicount = 0;
      }
      
      g_uilvdtflag = false;

  }
  
  g_psc910commdata->AI1_DATA_VALUE = g_sc910inputdata.AI1_DATA_VALUE;
  g_psc910commdata->AI2_DATA_VALUE = g_sc910inputdata.AI2_DATA_VALUE;
  g_psc910commdata->AI3_DATA_VALUE = g_sc910inputdata.AI3_DATA_VALUE;
  g_psc910commdata->AI_LVDT_DATA_VALUE = g_sc910inputdata.AI_LVDT_DATA_VALUE;
  g_psc910commdata->DI_DATA_VALUE = g_sc910inputdata.DI_DATA_VALUE;
  
  sc910_input_convert(&g_sc910inputdata, &g_sc910inputlogicdata);
  

}

void sc910_output()
{
  static uint32_t i = 0, j = 0;  
  
  sc910_output_convert(&g_sc910outputdata, &g_sc910outputlogicdata);
  
  g_psc910commdata->AO1_DATA_VALUE = g_sc910outputlogicdata.AO1_DATA_VALUE;
  g_sc910commdata.AO1_DATA_VALUE = g_sc910outputlogicdata.AO1_DATA_VALUE;
  
  g_psc910commdata->AO2_DATA_VALUE = g_sc910outputlogicdata.AO2_DATA_VALUE;
  g_sc910commdata.AO2_DATA_VALUE = g_sc910outputlogicdata.AO2_DATA_VALUE;
  
  g_psc910commdata->AO3_DATA_VALUE = g_sc910outputlogicdata.AO3_DATA_VALUE;
  g_sc910commdata.AO3_DATA_VALUE = g_sc910outputlogicdata.AO3_DATA_VALUE;
  
  g_psc910commdata->DO_DATA_VALUE = g_sc910outputlogicdata.DO_DATA_VALUE;
  
#if 1
  
  //驱动阀门
  
  if(g_sc910param.AO_CHSEL & LOW_BIT0)
  {
      DAC_DRV_Output(BOARD_DAC_DEMO_DAC_INSTANCE, g_sc910outputdata.AO1_DATA_VALUE);
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT1)
  {
      set_ao2_cv(g_sc910outputdata.AO2_DATA_VALUE);
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT2)
  {
      set_lvdt_cv(g_sc910outputdata.AO3_DATA_VALUE);
  }
  else
  {
      ;
  }
  
  //反馈输出
  
  if(g_sc910param.AO_CHSEL & HIGH_BIT0)
  {
      DAC_DRV_Output(BOARD_DAC_DEMO_DAC_INSTANCE, g_sc910outputdata.AO1_DATA_VALUE);
  }
  else if(g_sc910param.AO_CHSEL & HIGH_BIT1)
  {
      set_ao2_cv(g_sc910outputdata.AO2_DATA_VALUE);
  }
  else if(g_sc910param.AO_CHSEL & HIGH_BIT2)
  {
      set_lvdt_cv(g_sc910outputdata.AO3_DATA_VALUE);
  }
  else
  {
      ;
  }

  
  DO1 = g_sc910outputdata.DO_DATA_VALUE & 0x01;
  DO2 = (g_sc910outputdata.DO_DATA_VALUE >> 1) & 0x01;
  DO3 = (g_sc910outputdata.DO_DATA_VALUE >> 2) & 0x01;
  
#endif
  
#if 0
  DAC_DRV_Output(BOARD_DAC_DEMO_DAC_INSTANCE, 3000);
  //AO2_DIR = 1;
  set_lvdt_cv((LVDT_PWM_PERIOD/5)*3);
  set_ao2_cv((PWM1_PERIOD/5)*1);         
#endif
  
}


void sc910_iec()
{
  //need to modify.
  g_iLogic_offset = g_sc910param.OFFSET;
  
  //calc the logic diff
  
  //计算阀门控制逻辑输出
  g_iLogic_Kp = (float)g_sc910param.PID_K/1000;
  g_iLogic_realoutput = (long)(g_iLogic_Kp*(g_iLogic_given - g_iLogic_feedback) + g_iLogic_offset);
  
  if(g_iLogic_realoutput > GIVE_MAX_LOGIC)
  {
      g_iLogic_realoutput = GIVE_MAX_LOGIC;
  }
  else if(g_iLogic_realoutput < GIVE_MIN_LOGIC)
  {
      g_iLogic_realoutput = GIVE_MIN_LOGIC;
  }
  
  //根据输出配置选择输出
  if(g_sc910param.AO_CHSEL & LOW_BIT0)
  {
    g_sc910outputlogicdata.AO1_DATA_VALUE = g_iLogic_realoutput;
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT1)
  {
    g_sc910outputlogicdata.AO2_DATA_VALUE = g_iLogic_realoutput;
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT2)
  {
    g_sc910outputlogicdata.AO3_DATA_VALUE = g_iLogic_realoutput;
  }
  else
  {
    ;
  }
     
  //配置反馈输出
     
  if(g_sc910param.AO_CHSEL & HIGH_BIT0)
  {
    if(g_sc910param.AI_CHSEL & HIGH_BIT0)
    {
        g_sc910outputlogicdata.AO1_DATA_VALUE = g_sc910inputlogicdata.AI1_DATA_VALUE;
    }
    else if(g_sc910param.AI_CHSEL & HIGH_BIT1)
    {
        g_sc910outputlogicdata.AO1_DATA_VALUE = g_sc910inputlogicdata.AI2_DATA_VALUE;
    }
    else if(g_sc910param.AI_CHSEL & HIGH_BIT2)
    {
        g_sc910outputlogicdata.AO1_DATA_VALUE = g_sc910inputlogicdata.AI3_DATA_VALUE;
    }
    
  }
  else if(g_sc910param.AO_CHSEL & HIGH_BIT1)
  {
    if(g_sc910param.AI_CHSEL & HIGH_BIT0)
    {
        g_sc910outputlogicdata.AO2_DATA_VALUE = g_sc910inputlogicdata.AI1_DATA_VALUE;
    }
    else if(g_sc910param.AI_CHSEL & HIGH_BIT1)
    {
        g_sc910outputlogicdata.AO2_DATA_VALUE = g_sc910inputlogicdata.AI2_DATA_VALUE;
    }
    else if(g_sc910param.AI_CHSEL & HIGH_BIT2)
    {
        g_sc910outputlogicdata.AO2_DATA_VALUE = g_sc910inputlogicdata.AI3_DATA_VALUE;
    }
  }
  else if(g_sc910param.AO_CHSEL & HIGH_BIT2)
  {
    if(g_sc910param.AI_CHSEL & HIGH_BIT0)
    {
        g_sc910outputlogicdata.AO3_DATA_VALUE = g_sc910inputlogicdata.AI1_DATA_VALUE;
    }
    else if(g_sc910param.AI_CHSEL & HIGH_BIT1)
    {
        g_sc910outputlogicdata.AO3_DATA_VALUE = g_sc910inputlogicdata.AI2_DATA_VALUE;
    }
    else if(g_sc910param.AI_CHSEL & HIGH_BIT2)
    {
        g_sc910outputlogicdata.AO3_DATA_VALUE = g_sc910inputlogicdata.AI3_DATA_VALUE;
    }
  }
  else
  {
    ;
  }
  
  
}




void sc910_regulate()
{
  if(REGULATE_MODE == g_sc910param.g_ucRunMode)
  {
      if(g_sc910param.AO_CHSEL & (HIGH_BIT3 | HIGH_BIT2| LOW_BIT2))
      {
          enable_lvdt_pwm();    
      }
      else
      {
          disable_lvdt_pwm();
      }
  
      if(g_sc910param.AO_CHSEL & (HIGH_BIT1| LOW_BIT1))
      {
          enable_ao2_pwm();
      }
      else
      {
          disable_ao2_pwm();
      }
  }
}

void sc910_comm_process()
{
  //判断运行模式
  sc910_write_reg(&g_sc910param.g_ucRunMode, &g_sc910commdata.RUN_MODE, g_psc910commdata->RUN_MODE, g_sc910commdata.RUN_MODE);
  

  //整定模式下，记录整定值
  if(REGULATE_MODE == g_sc910param.g_ucRunMode)
  {
    sc910_regulate_write();
  } 
  
  //判断是否保存参数
  if((g_psc910commdata->SAVE_CMD == SAVE_PARAM) && (g_psc910commdata->SAVE_CMD != g_sc910commdata.SAVE_CMD))
  {
    static char save_time = 0;
    
    if(save_time < SAVE_TIMES_MAX)
    {
      g_iCom_savepara = SAVE_PARAM;
      save_time++;

    }
    else
    {
      g_psc910commdata->SAVE_CMD = SAVE_OVERTIMES;
      g_sc910commdata.SAVE_CMD = SAVE_OVERTIMES;
    }
    
  }
  
  
}

void sc910_input_convert(SC910InputData *raw, SC910InputData *logic)
{
   
  //logic->AI1_DATA_VALUE = ((raw->AI1_DATA_VALUE - (AD_MAX_VALUE/2)) * (GIVE_MAX_LOGIC - GIVE_MIN_LOGIC) / AD_MAX_VALUE);
  //logic->AI2_DATA_VALUE = ((raw->AI2_DATA_VALUE - (AD_MAX_VALUE/2)) * (GIVE_MAX_LOGIC - GIVE_MIN_LOGIC) / AD_MAX_VALUE);
  //logic->AI3_DATA_VALUE = ((raw->AI3_DATA_VALUE - (AD_MAX_VALUE/2)) * (GIVE_MAX_LOGIC - GIVE_MIN_LOGIC) / AD_MAX_VALUE);
  
  //AI1
  if(raw->AI1_DATA_VALUE < g_sc910param.AI1_MIN_VALUE)
  {
      logic->AI1_DATA_VALUE = GIVE_MIN_LOGIC;
  }
  else if(raw->AI1_DATA_VALUE < g_sc910param.AI1_MAX_VALUE)
  {
      logic->AI1_DATA_VALUE = GIVE_MIN_LOGIC + (GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)/(g_sc910param.AI1_MAX_VALUE - g_sc910param.AI1_MIN_VALUE)*(raw->AI1_DATA_VALUE - g_sc910param.AI1_MIN_VALUE); 
  }
  else
  {
      logic->AI1_DATA_VALUE = GIVE_MAX_LOGIC;
  }
  
  //AI2
  if(raw->AI2_DATA_VALUE < g_sc910param.AI2_MIN_VALUE)
  {
      logic->AI2_DATA_VALUE = GIVE_MIN_LOGIC;
  }
  else if(raw->AI2_DATA_VALUE < g_sc910param.AI2_MAX_VALUE)
  {
      logic->AI2_DATA_VALUE = GIVE_MIN_LOGIC + (GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)/(g_sc910param.AI2_MAX_VALUE - g_sc910param.AI2_MIN_VALUE)*(raw->AI2_DATA_VALUE - g_sc910param.AI2_MIN_VALUE); 
  }
  else
  {
      logic->AI2_DATA_VALUE = GIVE_MAX_LOGIC;
  }
  
  //AI3
  if(raw->AI3_DATA_VALUE < g_sc910param.AI3_MIN_VALUE)
  {
      logic->AI3_DATA_VALUE = GIVE_MIN_LOGIC;
  }
  else if(raw->AI3_DATA_VALUE < g_sc910param.AI3_MAX_VALUE)
  {
      logic->AI3_DATA_VALUE = GIVE_MIN_LOGIC + (GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)/(g_sc910param.AI3_MAX_VALUE - g_sc910param.AI3_MIN_VALUE)*(raw->AI3_DATA_VALUE - g_sc910param.AI3_MIN_VALUE); 
  }
  else
  {
      logic->AI3_DATA_VALUE = GIVE_MAX_LOGIC;
  }
  

  //MANUAL_MODE
  if(MANUAL_MODE == g_sc910param.g_ucRunMode)
  {
      ;
  }
  //REGULATE_MODE
  else if(REGULATE_MODE == g_sc910param.g_ucRunMode)
  {
      ;
  }
  //AUTO_MODE
  else
  {
      if(g_sc910param.AI_CHSEL == AI_CHSEL_AI1_GIVEN_AI2_FEEDBACK)
      {
          g_iLogic_given = logic->AI1_DATA_VALUE;
          g_iLogic_feedback = logic->AI2_DATA_VALUE;
      }
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI1_GIVEN_AI3_FEEDBACK)
      {
          g_iLogic_given = logic->AI1_DATA_VALUE;
          g_iLogic_feedback = logic->AI3_DATA_VALUE; 
      }
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI2_GIVEN_AI1_FEEDBACK)
      {
          g_iLogic_given = logic->AI2_DATA_VALUE;
          g_iLogic_feedback = logic->AI1_DATA_VALUE;   
      }
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI2_GIVEN_AI3_FEEDBACK)
      {
          g_iLogic_given = logic->AI2_DATA_VALUE;
          g_iLogic_feedback = logic->AI3_DATA_VALUE;   
      }    
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI3_GIVEN_AI1_FEEDBACK)
      {
          g_iLogic_given = logic->AI3_DATA_VALUE;
          g_iLogic_feedback = logic->AI1_DATA_VALUE;   
      } 
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI3_GIVEN_AI2_FEEDBACK)
      {
          g_iLogic_given = logic->AI3_DATA_VALUE;
          g_iLogic_feedback = logic->AI2_DATA_VALUE;   
      }  
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI1_GIVEN_LVDT_FEEDBACK)
      {
          g_iLogic_given = logic->AI1_DATA_VALUE;
          g_iLogic_feedback = logic->AI_LVDT_DATA_VALUE;   
      }
      else if(g_sc910param.AI_CHSEL == AI_CHSEL_AI2_GIVEN_LVDT_FEEDBACK)
      {
          g_iLogic_given = logic->AI2_DATA_VALUE;
          g_iLogic_feedback = logic->AI_LVDT_DATA_VALUE;   
      }
      //参数错误，阀位保持不动
      else
      {
          g_iLogic_feedback = g_iLogic_given;
      }  
  }
}



void sc910_output_convert(SC910OutputData *raw, SC910OutputData *logic)
{
  
  
  if(AUTO_MODE == g_sc910param.g_ucRunMode)
  {
    
    //限幅
    if((g_sc910param.AO_CHSEL == AO_CHSEL_AO1_DRIVE_AO2_FEEDBACK) || (g_sc910param.AO_CHSEL == AO_CHSEL_AO2_DRIVE_AO1_FEEDBACK))
    {
        sc910_limit_value(&logic->AO1_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC);
        sc910_limit_value(&logic->AO2_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC);
    }
    else if((g_sc910param.AO_CHSEL == AO_CHSEL_AO1_DRIVE_AO3_FEEDBACK) || (g_sc910param.AO_CHSEL == AO_CHSEL_AO3_DRIVE_AO1_FEEDBACK))
    {
        sc910_limit_value(&logic->AO1_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC);
        sc910_limit_value(&logic->AO3_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC);
    }
    else if((g_sc910param.AO_CHSEL == AO_CHSEL_AO2_DRIVE_AO3_FEEDBACK) || (g_sc910param.AO_CHSEL == AO_CHSEL_AO3_DRIVE_AO2_FEEDBACK))
    {
        sc910_limit_value(&logic->AO2_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC);
        sc910_limit_value(&logic->AO3_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC);
    }
    //参数错误
    else
    {
        ;
    }
    
#if 0
    //限幅
    if((g_sc910param.AO_CHSEL == AO_CHSEL_AO1_DRIVE_AO2_FEEDBACK) || (g_sc910param.AO_CHSEL == AO_CHSEL_AO2_DRIVE_AO1_FEEDBACK))
    {
        sc910_limit_value(&logic->AO1_DATA_VALUE, g_sc910param.AO1_MAX_VALUE, g_sc910param.AO1_MIN_VALUE);
        sc910_limit_value(&logic->AO2_DATA_VALUE, g_sc910param.AO2_MAX_VALUE, g_sc910param.AO2_MIN_VALUE);
    }
    else if((g_sc910param.AO_CHSEL == AO_CHSEL_AO1_DRIVE_AO3_FEEDBACK) || (g_sc910param.AO_CHSEL == AO_CHSEL_AO3_DRIVE_AO1_FEEDBACK))
    {
        sc910_limit_value(&logic->AO1_DATA_VALUE, g_sc910param.AO1_MAX_VALUE, g_sc910param.AO1_MIN_VALUE);
        sc910_limit_value(&logic->AO3_DATA_VALUE, g_sc910param.AO3_MAX_VALUE, g_sc910param.AO3_MIN_VALUE);
    }
    else if((g_sc910param.AO_CHSEL == AO_CHSEL_AO2_DRIVE_AO3_FEEDBACK) || (g_sc910param.AO_CHSEL == AO_CHSEL_AO3_DRIVE_AO2_FEEDBACK))
    {
        sc910_limit_value(&logic->AO2_DATA_VALUE, g_sc910param.AO2_MAX_VALUE, g_sc910param.AO2_MIN_VALUE);
        sc910_limit_value(&logic->AO3_DATA_VALUE, g_sc910param.AO3_MAX_VALUE, g_sc910param.AO3_MIN_VALUE);
    }
    //参数错误
    else
    {
        ;
    }
#endif
    
  }
  //手动模式下，输出来自Modbus上位机
  else if(MANUAL_MODE == g_sc910param.g_ucRunMode)
  {
    if(g_sc910param.AO_CHSEL & LOW_BIT0)
    {
        sc910_force_value(&logic->AO1_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC, g_psc910commdata->AO1_DATA_VALUE, g_sc910commdata.AO1_DATA_VALUE);
    }
    else if(g_sc910param.AO_CHSEL & LOW_BIT1)
    {
        sc910_force_value(&logic->AO2_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC, g_psc910commdata->AO2_DATA_VALUE, g_sc910commdata.AO2_DATA_VALUE); 
    }
    else if(g_sc910param.AO_CHSEL & LOW_BIT2)
    {
        sc910_force_value(&logic->AO3_DATA_VALUE, GIVE_MAX_LOGIC, GIVE_MIN_LOGIC, g_psc910commdata->AO3_DATA_VALUE, g_sc910commdata.AO3_DATA_VALUE);
    }
    else
    {
      //参数错误

    }
    
  }
  
  
  if(g_sc910param.AO_CHSEL & LOW_BIT0)
  {
      raw->AO1_DATA_VALUE = g_sc910param.AO1_MIN_VALUE + (logic->AO1_DATA_VALUE - GIVE_MIN_LOGIC)*(g_sc910param.AO1_MAX_VALUE - g_sc910param.AO1_MIN_VALUE)/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC);
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT1)
  {
      raw->AO2_DATA_VALUE = g_sc910param.AO2_MIN_VALUE + (logic->AO2_DATA_VALUE - GIVE_MIN_LOGIC)*(g_sc910param.AO2_MAX_VALUE - g_sc910param.AO2_MIN_VALUE)/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC);
   }
  else if(g_sc910param.AO_CHSEL & LOW_BIT2)
  {
      raw->AO3_DATA_VALUE = g_sc910param.AO3_MIN_VALUE + (logic->AO3_DATA_VALUE - GIVE_MIN_LOGIC)*(g_sc910param.AO3_MAX_VALUE - g_sc910param.AO3_MIN_VALUE)/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC);
   }
  else
  {
      //raw->AO2_DATA_VALUE = (int)(((float)logic->AO2_DATA_VALUE/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)) * PWM_MAX_VALUE) + PWM_MAX_VALUE/2;
  }
  
#if 0
  //转换成实际输出
  if(g_sc910param.AO_CHSEL & LOW_BIT0)
  {
      raw->AO1_DATA_VALUE = (int)(((float)logic->AO1_DATA_VALUE/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)) * DA_MAX_VALUE) + DA_MAX_VALUE/2;
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT1)
  {
      raw->AO2_DATA_VALUE = (int)(((float)logic->AO2_DATA_VALUE/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)) * PWM_MAX_VALUE) + PWM_MAX_VALUE/2;
  }
  else if(g_sc910param.AO_CHSEL & LOW_BIT2)
  {
      raw->AO3_DATA_VALUE = (int)(((float)logic->AO3_DATA_VALUE/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)) * PWM_MAX_VALUE) + PWM_MAX_VALUE/2;
  }
  else
  {
      //raw->AO2_DATA_VALUE = (int)(((float)logic->AO2_DATA_VALUE/(GIVE_MAX_LOGIC - GIVE_MIN_LOGIC)) * PWM_MAX_VALUE) + PWM_MAX_VALUE/2;
  }
  
#endif
  
}

void sc910_limit_value(int16_t *dst, int16_t high, int16_t low)
{
    if(*dst > high)
    {
        *dst = high;
    }
    else if(*dst < low)
    {
        *dst = low;
    }
}

void sc910_force_value(int16_t *dst, int16_t high, int16_t low, int16_t new_value,int16_t retain)
{
      if((*dst < GIVE_MAX_LOGIC) && (*dst > GIVE_MIN_LOGIC))
      {
          *dst = new_value;
      }
      //上位机参数有误，继续使用之前的数据
      else
      {
          *dst = retain;
      }  
}

void sc910_write_reg(int16_t *param, int16_t *commdata, int16_t new_value, int16_t old_value)
{
    if(new_value != old_value)
    {
        *param = new_value;
        *commdata = new_value;
    }
}

void sc910_regulate_write()
{
    sc910_write_reg(&g_sc910param.AI1_MAX_VALUE, &g_sc910commdata.AI1_MAX_VALUE, g_psc910commdata->AI1_MAX_VALUE, g_sc910commdata.AI1_MAX_VALUE);
    sc910_write_reg(&g_sc910param.AI1_MIN_VALUE, &g_sc910commdata.AI1_MIN_VALUE, g_psc910commdata->AI1_MIN_VALUE, g_sc910commdata.AI1_MIN_VALUE);
         
    sc910_write_reg(&g_sc910param.AI2_MAX_VALUE, &g_sc910commdata.AI2_MAX_VALUE, g_psc910commdata->AI2_MAX_VALUE, g_sc910commdata.AI2_MAX_VALUE);
    sc910_write_reg(&g_sc910param.AI2_MIN_VALUE, &g_sc910commdata.AI2_MIN_VALUE, g_psc910commdata->AI2_MIN_VALUE, g_sc910commdata.AI2_MIN_VALUE);
        
    sc910_write_reg(&g_sc910param.AI3_MAX_VALUE, &g_sc910commdata.AI3_MAX_VALUE, g_psc910commdata->AI3_MAX_VALUE, g_sc910commdata.AI3_MAX_VALUE);
    sc910_write_reg(&g_sc910param.AI3_MIN_VALUE, &g_sc910commdata.AI3_MIN_VALUE, g_psc910commdata->AI3_MIN_VALUE, g_sc910commdata.AI3_MIN_VALUE);

    sc910_write_reg(&g_sc910param.AO1_MAX_VALUE, &g_sc910commdata.AO1_MAX_VALUE, g_psc910commdata->AO1_MAX_VALUE, g_sc910commdata.AO1_MAX_VALUE);
    sc910_write_reg(&g_sc910param.AO1_MIN_VALUE, &g_sc910commdata.AO1_MIN_VALUE, g_psc910commdata->AO1_MIN_VALUE, g_sc910commdata.AO1_MIN_VALUE);  
    
    sc910_write_reg(&g_sc910param.AO2_MAX_VALUE, &g_sc910commdata.AO2_MAX_VALUE, g_psc910commdata->AO2_MAX_VALUE, g_sc910commdata.AO2_MAX_VALUE);
    sc910_write_reg(&g_sc910param.AO2_MIN_VALUE, &g_sc910commdata.AO2_MIN_VALUE, g_psc910commdata->AO2_MIN_VALUE, g_sc910commdata.AO2_MIN_VALUE);  
    
    sc910_write_reg(&g_sc910param.AO3_MAX_VALUE, &g_sc910commdata.AO3_MAX_VALUE, g_psc910commdata->AO3_MAX_VALUE, g_sc910commdata.AO3_MAX_VALUE);
    sc910_write_reg(&g_sc910param.AO3_MIN_VALUE, &g_sc910commdata.AO3_MIN_VALUE, g_psc910commdata->AO3_MIN_VALUE, g_sc910commdata.AO3_MIN_VALUE);  
    
    sc910_write_reg(&g_sc910param.AO_CHSEL, &g_sc910commdata.AO_CHSEL, g_psc910commdata->AO_CHSEL, g_sc910commdata.AO_CHSEL);  
    sc910_write_reg(&g_sc910param.AI_CHSEL, &g_sc910commdata.AI_CHSEL, g_psc910commdata->AI_CHSEL, g_sc910commdata.AI_CHSEL);
    sc910_write_reg(&g_sc910param.PID_K, &g_sc910commdata.PID_K, g_psc910commdata->PID_K, g_sc910commdata.PID_K);  
    sc910_write_reg(&g_sc910param.OFFSET, &g_sc910commdata.OFFSET, g_psc910commdata->OFFSET, g_sc910commdata.OFFSET);

}


void seek_max_min(uint16_t * max, uint16_t * min, uint16_t * max_pos, uint16_t * min_pos, uint16_t *pbuf, uint16_t len)
{
    int i = 0;
    
    *max = pbuf[0];
    *min = pbuf[0];
    for( i = 1; i < len; i++)
    {
        if(*max < pbuf[i])
        {
            *max = pbuf[i];
            *max_pos = i;
        }
        
        if(*min > pbuf[i])
        {
            *min = pbuf[i];
            *min_pos = i;
        }
    }
    
}


