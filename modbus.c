#include "modbus.h"
#include <string.h>
#include "adc_hw_trigger.h"

unsigned char ucHMI0Size=0;


char g_cUART0Clear;

RxBuffer  s_RxUART0Buf;
TxBuffer  s_TxUART0Buf;

unsigned char g_ucMbuf[MODBUS_MAX_BUFFER];


unsigned char g_ucSend0Len = 0;
unsigned long g_ucUART0ModbusTime = 0;
unsigned char g_ucUART0ModbusFlag = 0;


unsigned char Local_Address=0x1;
unsigned char g_ucCC28Int = 0;

unsigned char HMIRecvBuffer[MODBUS_MAX_BUFFER];	//changed by liqi
unsigned char HMISendBuffer[MODBUS_MAX_BUFFER];	//changed by liqi

static const unsigned char ClearMask[8] = {0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};
static const unsigned char SetMask[8]   = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
static const unsigned char BitClearHiMask[8]  = {0x00,0x01,0x03,0x07,0x0f,0x1f,0x3f,0x7f};
static const unsigned char BitClearLowMask[8] = {0xff,0xfe,0xfc,0xf8,0xf0,0xe0,0xc0,0x80};


unsigned char ModbusReceive(unsigned char *pbyData, unsigned short uCount,unsigned char port)
{
  if ( HMI_ACCESS_HEADER(pbyData)->bSlaveAdd == (unsigned char)Local_Address ) /*Recv HMI data, master header*/
  {
    switch (HMI_ACCESS_HEADER(pbyData)->bFuncCode)
    {
    case HMI_READ_DIGITAL_OUTPUT:
    case HMI_READ_DIGITAL_INPUT:
    case HMI_READ_ANOLOG_OUTPUT:
    case HMI_READ_ANOLOG_INPUT:
    case HMI_FORCE_SINGLE_DIGITAL_OUTPUT:
    case HMI_FORCE_SINGLE_ANOLOG_OUTPUT:
      {
        if (port == 0)
        {
          ucHMI0Size = 8;
        }
        else
        {
          return false;
        }
        break;
      }
    case HMI_FORCE_MUL_DIGITAL_OUTPUT:
    case HMI_FORCE_MUL_ANOLOG_OUTPUT:
      if (uCount < 7)
      {
        return false;  // continue receive data
      }
      if (port == 0)
      {
        ucHMI0Size = *(pbyData+6) + 9;
      }
      else
      {
        return false;
      }
      
      break;
    default:   // shift data
      
      if (port == 0)
      {
        g_cUART0Clear = true;
      }
      else
      {
        return false;
      }
      
      return true;
    }// end switch(FuncCode)
    
    if (port == 0)
    {
      if (uCount < ucHMI0Size)
      {
        return false;  // continue receive data
      }
      else// recv data ok
      {
        g_ucCC28Int |= 1;
        g_cUART0Clear = false;        // recv new data
        return true;
      }
    }
    else
    {
      return true;
    }
    
    //return false;
  }
  
  return false;
  
}

void CstModbusSendAck(void)
{
  if (g_ucCC28Int & 0x01 )
  {
    
    SendAckToMaster(0);
    g_ucCC28Int &= ~0x01;
    
    if ( !g_cUART0Clear )
    {
      // Receive over
      s_RxUART0Buf.uCount = 0;
    }
    /*
    else
    {
    // First Character was omitted
    unsigned short i;
    for (i = 0; i < s_RxUART0Buf.uCount; i++)
    {
    s_RxUART0Buf.byData[i] = s_RxUART0Buf.byData[i + 1];
  }
    s_RxUART0Buf.uCount = s_RxUART0Buf.uCount - 1;
  }*/
  }
  
}

void SendAckToMaster(unsigned int port)
{
  unsigned char *pbyData;
  unsigned int wCrcCheck, wCrcReal;
  unsigned char ucSendLen, i;
  
  if (port == 0)
  {
    pbyData= (unsigned char*)s_RxUART0Buf.byData;
    wCrcCheck = GenCrcCode(pbyData, ucHMI0Size-2);
    wCrcReal = MAKEWORD(*(pbyData+(ucHMI0Size-1)), *(pbyData+(ucHMI0Size-2)));
    if ( wCrcCheck == wCrcReal )
    {
      memcpy(HMIRecvBuffer, pbyData, ucHMI0Size);
      g_ucSend0Len = CstProcessHMIService();
      memcpy((void*)s_TxUART0Buf.byData, &HMISendBuffer[0], g_ucSend0Len);
      Wait35Char(port);
      
      /*
      for(i = 0; i< g_ucSend0Len;i++)
      {	
      while(BR_UART_S1_TC(UART0_BASE) == 0);
      UART_HAL_Putchar(UART0_BASE, s_TxUART0Buf.byData[i]);
      
    }
      //send_uart_dma0(g_ucSend0Len);
      //UART_DRV_SendDataBlocking(0, s_TxUART0Buf.byData, g_ucSend0Len,OSA_WAIT_FOREVER);
      
      g_ucSend0Len = 0;
      */		
    }
    
  }
  else
  {
    return;
  }
  
}


unsigned int GenCrcCode(unsigned char *datapt,unsigned char bytecount)
{
  unsigned int  Reg16=0xFFFF,mval=0xA001;  //0xA001=1010000000000001
  int   i;
  //	WORD  chkcode;
  char  j,flg; 
  for(i=0; i<bytecount; i++)
  {
    Reg16^=*(datapt+i);
    for(j=0;j<8;j++)
    {
      flg = 0;
      flg = (char)(Reg16&0x0001);  //get b0 value
      Reg16>>=1;         //right shift 1 bit ,set b15=0
      if(flg==1)
      {
        Reg16=Reg16^mval;
      }
    }
  }
  //	printf("crccode=%x",Reg16);
  return(Reg16);
}


void Wait35Char(unsigned int port)
{
  
  if(port == 0)
  {
    g_ucUART0ModbusTime = get_waittime();
    g_ucUART0ModbusFlag = MODBUS_SEND;
  }
  else
  {
    ;
  }
  
}

unsigned char CstProcessHMIService()
{
  unsigned int wByteAddOffset, wChannelNum;
  unsigned char ucStartBit, ucWriteByteNum;
  unsigned char ucDataSize,ucSendDataLen,i,temp;  //j,
  unsigned short CrcReal;
  unsigned char *pIn, *pOut;
  //haoli added
  unsigned int wReadBitNum,wReturnByteNum;
  
  HMISendBuffer[0] = (unsigned char)Local_Address;//PLC address
  HMISendBuffer[1] = HMIRecvBuffer[1];//Function code
  
  wByteAddOffset = MAKEWORD(HMIRecvBuffer[2],HMIRecvBuffer[3]);
  
  switch (HMI_ACCESS_HEADER(HMIRecvBuffer)->bFuncCode)
  {
  case HMI_READ_DIGITAL_OUTPUT://only read one channel
    {
      //haoli added for multi_do
      wReadBitNum = MAKEWORD(HMIRecvBuffer[4],HMIRecvBuffer[5]);
      wReturnByteNum = wReadBitNum / 8;
      if ( wReadBitNum % 8 != 0 )
      {
        wReturnByteNum++;
      }
      HMISendBuffer[2] = (unsigned char)wReturnByteNum;//byte counter
      ucSendDataLen = 5 + (unsigned char)wReturnByteNum;		
      
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;	
        ucStartBit = (unsigned char)(wByteAddOffset & 0x0007);
        wByteAddOffset = wByteAddOffset >> 3;
        if( (ucStartBit==0)&&(wReadBitNum%8==0) )//full bytes
        {
          for ( i=0; i<wReturnByteNum; i++ )
          {
            HMISendBuffer[3+i] = g_ucMbuf[wByteAddOffset+i];		//yangliang
          }
          break;
        }
        
        if ( wReadBitNum == 1 )
        {
          HMISendBuffer[3] =  (g_ucMbuf[wByteAddOffset] >> ucStartBit) & 0x01;//data
          break;
        }
        else
        {
          for (i=0; i<wReturnByteNum; i++)
          {
            HMISendBuffer[3+i] = g_ucMbuf[wByteAddOffset+i] >> ucStartBit;
            temp = g_ucMbuf[wByteAddOffset+i+1] << (8-ucStartBit);
            HMISendBuffer[3+i] |= temp;
            if( wReadBitNum < 8 )
            {
              HMISendBuffer[3+i] &= BitClearHiMask[wReadBitNum];
              break;
            }
            else
            {
              wReadBitNum -= 8;
            }
          }
        }                        
      }
      else
      {
        ;
      }
      
      break;
      
      
    }
    
  case HMI_READ_DIGITAL_INPUT:
    {
      //haoli added for multi_di
      wReadBitNum = MAKEWORD(HMIRecvBuffer[4],HMIRecvBuffer[5]);
      wReturnByteNum = wReadBitNum / 8;
      if ( wReadBitNum % 8 != 0 )
      {
        wReturnByteNum++;
      }
      HMISendBuffer[2] = (unsigned char)wReturnByteNum;//byte counter
      ucSendDataLen = 5 + (unsigned char)wReturnByteNum;
      
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;		
      }
      else//I Area
      {
        break;
      }
      
      ucStartBit = (unsigned char)(wByteAddOffset & 0x0007);
      wByteAddOffset = wByteAddOffset >> 3;
      
      //haoli added for multi_input
      if ( (ucStartBit==0)&&(wReadBitNum%8==0) )//full bytes
      {
        for ( i=0; i<wReturnByteNum; i++ )
        {
          HMISendBuffer[3+i] = g_ucMbuf[wByteAddOffset+i];
        }
        break;
      }
      
      if ( wReadBitNum == 1 )
      {
        HMISendBuffer[3] = (g_ucMbuf[wByteAddOffset] >> ucStartBit) & 0x01;//data
        break;
      }
      else
      {
        for (i=0; i<wReturnByteNum; i++)
        {
          HMISendBuffer[3+i] = g_ucMbuf[wByteAddOffset+i] >> ucStartBit;
          temp = g_ucMbuf[wByteAddOffset+i+1] << (8-ucStartBit);
          HMISendBuffer[3+i] |= temp;
          if( wReadBitNum < 8 )
          {
            HMISendBuffer[3+i] &= BitClearHiMask[wReadBitNum];
            break;
          }
          else
          {
            wReadBitNum -= 8;
          }
        }
      }
      break;
      
      
    }
    
  case HMI_READ_ANOLOG_OUTPUT:
    {
      wChannelNum = MAKEWORD(HMIRecvBuffer[4],HMIRecvBuffer[5]);
      ucDataSize = (unsigned char)(wChannelNum*2);
      HMISendBuffer[2] = ucDataSize;//byte counter
      
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;
        wByteAddOffset *= 2;
        
        for (i=0; i<ucDataSize; i=i+2)
        {
          HMISendBuffer[3+i] = g_ucMbuf[i + 1];//high byte
          HMISendBuffer[4+i] = g_ucMbuf[i];//low byte
        }
        
      }
      
      ucSendDataLen = 5 + ucDataSize;
      break;
      
    }
  case HMI_READ_ANOLOG_INPUT:
    {
      wChannelNum = MAKEWORD(HMIRecvBuffer[4],HMIRecvBuffer[5]);
      ucDataSize = (unsigned char)(wChannelNum*2);
      HMISendBuffer[2] = ucDataSize;//byte counter
      
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;	
        wByteAddOffset *= 2;
        for (i=0; i<ucDataSize; i=i+2)
        {
          HMISendBuffer[3+i] = g_ucMbuf[i + 1];//high byte
          HMISendBuffer[4+i] = g_ucMbuf[i];//low byte
        }
        
      }
      
      ucSendDataLen = 5 + ucDataSize;
      break;
      
    }
    
    
  case HMI_FORCE_SINGLE_DIGITAL_OUTPUT:
    {
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;
        ucStartBit = (unsigned char)(wByteAddOffset & 0x0007);
        wByteAddOffset = wByteAddOffset >> 3;
        if( wByteAddOffset < MODBUS_MAX_BUFFER )
        {
          if(( HMIRecvBuffer[4] == 0xFF )&&(HMIRecvBuffer[5] == 0x00))
          {
            g_ucMbuf[wByteAddOffset] |= SetMask[ucStartBit];
          }
          if(( HMIRecvBuffer[4] == 0x00) &&(HMIRecvBuffer[5] == 0x00))
          {
            g_ucMbuf[wByteAddOffset] &= ClearMask[ucStartBit];
          }
        }			
      }
      
      
      for (i=0; i<6; i++)
      {
        HMISendBuffer[i] = HMIRecvBuffer[i];
      }
      ucSendDataLen = 8;
      break;
      
    }
    
  case HMI_FORCE_SINGLE_ANOLOG_OUTPUT:
    {
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;		
        //convert to mode of byte address
        wByteAddOffset *= 2; 
        if( wByteAddOffset < MODBUS_MAX_BUFFER )  //超限判断需要重新处理
        {
          g_ucMbuf[wByteAddOffset] = HMIRecvBuffer[5];//low byte
          g_ucMbuf[wByteAddOffset + 1] = HMIRecvBuffer[4];//high byte
        }
      }
      
      //fill HMI SendBuffer
      for (i=0; i<6; i++)
      {
        HMISendBuffer[i] = HMIRecvBuffer[i];
      }
      ucSendDataLen = 8;
      break;
      
    }
    
  case HMI_FORCE_MUL_DIGITAL_OUTPUT:
    {
      if( wByteAddOffset >= 3000 ) //M Area
      {
        wByteAddOffset -= 3000;
        if(((wByteAddOffset/8)+HMIRecvBuffer[6]) > MODBUS_MAX_BUFFER)
        {
          break;
        }
      }
      else
      {
        return 0;
      }		
      
      
      ucStartBit = (unsigned char)(wByteAddOffset & 0x0007);
      wByteAddOffset = wByteAddOffset >> 3;
      
      ucWriteByteNum = HMIRecvBuffer[6];
      wReadBitNum = MAKEWORD(HMIRecvBuffer[4],HMIRecvBuffer[5]);
      
      if( ucStartBit==0 )
      {
        for(i=0; i<ucWriteByteNum; i++)
        {
          if( wReadBitNum >= 8 )
          {
            g_ucMbuf[wByteAddOffset+i] = HMIRecvBuffer[7+i];
            wReadBitNum -= 8;
          }
          else
          {
            g_ucMbuf[wByteAddOffset+i] &= BitClearLowMask[wReadBitNum];
            HMIRecvBuffer[7+i] &= BitClearHiMask[wReadBitNum];//清除高位 add by liqi
            g_ucMbuf[wByteAddOffset+i] |= HMIRecvBuffer[7+i];
          }
        }
      }//end if
      else
      {
        for(i=0; i<ucWriteByteNum; i++)
        {
          if( wReadBitNum >= 8 )
          {
            //first byte
            g_ucMbuf[wByteAddOffset+i] &= BitClearHiMask[ucStartBit];//clear Bit after start bit
            temp = HMIRecvBuffer[7+i] << ucStartBit;
            g_ucMbuf[wByteAddOffset+i] |= temp;
            //second byte
            g_ucMbuf[wByteAddOffset+i+1] &= BitClearLowMask[ucStartBit];
            temp = HMIRecvBuffer[7+i] >> (8-ucStartBit);
            g_ucMbuf[wByteAddOffset+i+1] |= temp;
            wReadBitNum -= 8;
          }
          else 
          {
            if( wReadBitNum < (8-ucStartBit))
            {
              temp = pOut[wByteAddOffset+i] >> ucStartBit;
              temp &= BitClearLowMask[wReadBitNum];
              HMIRecvBuffer[7+i] &=  BitClearHiMask[wReadBitNum];//add by liqi
              temp |= HMIRecvBuffer[7+i];
              temp <<= ucStartBit;
              g_ucMbuf[wByteAddOffset+i] &= BitClearHiMask[ucStartBit];
              g_ucMbuf[wByteAddOffset+i] |= temp;
            }
            else
            {
              //the first byte
              g_ucMbuf[wByteAddOffset+i] &= BitClearHiMask[ucStartBit];
              temp = HMIRecvBuffer[7+i] << ucStartBit;
              g_ucMbuf[wByteAddOffset+i] |= temp;
              //the second byte
              //pOut[wByteAddOffset+i+1] &= BitClearHiMask[wReadBitNum-(8-ucStartBit)];
              g_ucMbuf[wByteAddOffset+i+1] &= BitClearLowMask[wReadBitNum-(8-ucStartBit)];//add by liqi
              temp = HMIRecvBuffer[7+i] >> (8-ucStartBit);
              temp &= BitClearHiMask[wReadBitNum-(8-ucStartBit)];//add by liqi
              g_ucMbuf[wByteAddOffset+i+1] |= temp;
            }
          }
        }//end for
      }//end else     
      
      for (i=0; i<6; i++)
      {
        HMISendBuffer[i] = HMIRecvBuffer[i];
      }
      ucSendDataLen = 8;
      break;
      
    }
    
  case HMI_FORCE_MUL_ANOLOG_OUTPUT:
    {
      ucDataSize = HMIRecvBuffer[6];
      wChannelNum = MAKEWORD(HMIRecvBuffer[4],HMIRecvBuffer[5]);
      
      if(( wByteAddOffset >= 3000 ) && (wByteAddOffset < (3000 + MODBUS_MAX_BUFFER)))//M Area
      {
        wByteAddOffset -= 3000;		
        //convert to mode of byte address
        wByteAddOffset *= 2; 								
        //if( wByteAddOffset < DATAIDX_MEMORY_SIZE )
        if( (wByteAddOffset+wChannelNum*2) < MODBUS_MAX_BUFFER )	//判断结尾是否越界
        {
          for(i=0; i<ucDataSize; i=i+2)
          {
            g_ucMbuf[i] = HMIRecvBuffer[8+i];//low byte
            g_ucMbuf[i + 1] = HMIRecvBuffer[7+i];//high byte
          }
        }
        
      }
            
      
      //fill HMI SendBuffer
      ucSendDataLen = 8;
      for (i=0; i<6; i++)
      {
        HMISendBuffer[i]= HMIRecvBuffer[i];
      }
      break;
      
    }
    
  default:
    ucSendDataLen = 6;
    break;
  }
  CrcReal = GenCrcCode(HMISendBuffer, ucSendDataLen-2);
  HMISendBuffer[ucSendDataLen-2] = (unsigned char)(CrcReal & 0x00FF);	//low byte
  HMISendBuffer[ucSendDataLen-1] = (unsigned char)(CrcReal >> 8); //high byte
  return ucSendDataLen;
}




