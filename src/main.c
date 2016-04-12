#include <string.h>
#include <stdio.h>
#include "stm32f0xx.h"
#include "system_stm32f0xx.h"
#include "ff.h"

#define F_CPU           48000000UL
#define TimerTick	F_CPU/1000-1 // 1 kHz
#define PWROFF          GPIOB -> BSRR = GPIO_BSRR_BR_3;
#define PWRHOLD         GPIOB -> BSRR = GPIO_BSRR_BS_3;
#define MODEMON         GPIOA -> BSRR = GPIO_BSRR_BS_8;\
                            Delay(5000000);\
                            GPIOA -> BSRR = GPIO_BSRR_BR_8;                       
#define LEDOFF          GPIOB -> BSRR = GPIO_BSRR_BR_15;
#define LEDON           GPIOB -> BSRR = GPIO_BSRR_BS_15;
#define LEDXOR          GPIOB -> ODR ^= GPIO_ODR_15;
#define BUFFSIZE        255
                            
#define time            't'
#define set             's'
#define get             'g'
                           

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  /*    Откройте свойства проекта (Options...) -> General Options -> 
        Library Configuration -> Library: поставьте Full.*/
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

//extern void disk_timerproc(void);

//DWORD AccSize;				/* Work register for fs command */
//WORD AccFiles, AccDirs;
//FILINFO Finfo;
//FRESULT result;
__IO uint8_t nRead, nWritten;   // Возвращаемые значения прочитаных и записанных байт на/с SD карту.
//DWORD ofs_crs = 0;
//unsigned char sh;

/*Буффер для записис данных на SD карту. Помещается 
  информация для записи. Каждая запись с буфера 
  производится в новую строку в файле. Длина строки не превышает 
  размер 255 символов (байт)*/
char modemRxBuff[BUFFSIZE];
char modemTxBuff[BUFFSIZE];
char cmdModemSet = 0;

char consoleRxBuff[BUFFSIZE];
char consoleTxBuff[BUFFSIZE];
char cmdConsoleSet = 0;

__IO uint8_t cntModemRx = 0;
uint8_t cntConsoleRx = 0;
uint8_t readyModem = 0; 
uint8_t sendData = 0;
uint32_t data1 = 0;
uint32_t data2 = 0;

__IO uint8_t ALARM_Occured = 0;

//__IO uint8_t readyModem= 0;
uint8_t statusPwrModem = 0;
uint8_t startInitialize = 0;

				/* File system object for each logical drive */
				/* File objects */
//DIR dir;					/* Directory object */
uint32_t cntTimeMs;
__IO uint32_t cntTimeDelay;

//uint32_t counterPositive = 1654; //0;
//uint32_t counterNegative = 988;  //0;

uint32_t counterPositive = 0;
uint32_t counterNegative = 0;

uint32_t timeOffDelay = 0;
//------------------------------------------------------------------------------
void Delay(unsigned int Val);

static void RCC_Config(void);
static void USART1_Config(void);
static void USART2_Config(void);
static void RTC_Config(void);
static void GPIO_Config(void);
static void RTC_AlarmConfig(void);
static void EXTI4_15_Config(void);

static void SendCmdToModem(char *buffRx);
static void SendCmdToConsole(char *buffRx);
static void SetNextStartTime(RTC_TimeTypeDef  RTC_currentTimeStruct, uint8_t hours, uint8_t minutes, uint8_t seconds);
static void SetCurrentTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
static void SetCurrentDate(uint8_t date, uint8_t month, uint8_t year);
static void ClearBuffer(char* buff);

static void WaitTickTime(uint32_t *waitTime);

//void RTC_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void SysTick_Handler(void);
//-------------------------------------------------------------------
//void RTC_IRQHandler(void)
//static void USART_SendString();
void put_rc(FRESULT rc);
int StrToInt(char *ptr);
//------------------------------------------------------------------------------
int main( void) 
{
  /* Начальная конфигурация устройства */
  SystemInit();
  RCC_Config(); 
  GPIO_Config(); 
  
  /* После конфигурации GPIO сразу включить удержание питания!!! */
  PWRHOLD;
  
  /* Внешние прерывания */
  EXTI4_15_Config();
  
  SysTick_Config(SystemCoreClock / 1000);
  USART1_Config();
  USART2_Config();
  RTC_Config();
  RTC_AlarmConfig();
  
  /* Установка следующего времени запуска устроуства по умолчанию, 30 сек */
  //SetNextStartTime();
  
  //printf("Start!");
  LEDON;
  __enable_irq ();
  SendCmdToConsole("\r\n> ");
 
  while(1) 
  {
/*******************************************************************************
 *                           Console command                                   *
 *                                                                             *
 *******************************************************************************/
    if (cmdConsoleSet)
    {
    /******************************************************************************
                            Set time
    *******************************************************************************/
        if (strncmp(consoleRxBuff, "set time", 8) == 0 && cmdConsoleSet && cntConsoleRx == 17)
        {
          int hours = 0; 
          int minutes = 0;
          int seconds = 0;
          sscanf(consoleRxBuff + 9, "%d %d %d", &hours, &minutes, &seconds);
          SetCurrentTime(hours, minutes, seconds); //Arguments: 1st - hours, 2nd - minutes, 3rd - seconds
    
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
          printf("\r\nOK");
          SendCmdToConsole("\r\n> ");
        } 
    /******************************************************************************
                            Set date
    *******************************************************************************/
        if (strncmp(consoleRxBuff, "set date", 8) == 0 && cmdConsoleSet && cntConsoleRx == 17)
        {
          int date = 0; 
          int month = 0;
          int year = 0;
          sscanf(consoleRxBuff + 9, "%d %d %d", &date, &month, &year);
          SetCurrentDate(date, month, year); //Arguments: 1st - hours, 2nd - minutes, 3rd - seconds       
   
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
          printf("\r\nOK");
          SendCmdToConsole("\r\n> ");
        }
        
        if (strncmp(consoleRxBuff, "get time", 8) == 0 && cmdConsoleSet && cntConsoleRx == 8)
        {
          
          /*  */
          RTC_DateTypeDef RTC_CurrentDateStructure;
          RTC_GetDate(RTC_Format_BIN, &RTC_CurrentDateStructure);
          
          /*Получить время для установки следующего времени запуска */
          RTC_TimeTypeDef  RTC_gettingTimeStruct;
          RTC_GetTime(RTC_Format_BIN, &RTC_gettingTimeStruct);
          
          /* Отправить текущее время перед выключением */
          printf("\r\n\tDate:\t%02u/%02u/%02d", RTC_CurrentDateStructure.RTC_Date,
                                                RTC_CurrentDateStructure.RTC_Month,
                                                RTC_CurrentDateStructure.RTC_Year
                                              );
          printf("\r\n\tTime:\t%02u:%02u:%02d", RTC_gettingTimeStruct.RTC_Hours, 
                                                RTC_gettingTimeStruct.RTC_Minutes, 
                                                RTC_gettingTimeStruct.RTC_Seconds
                                               );
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
          SendCmdToConsole("\r\n> ");
        } 
        /******************************************************************************
                            Send sequense of pulses
        *******************************************************************************/
        if (strncmp(consoleRxBuff, "pulses", 6) == 0 && cmdConsoleSet && cntConsoleRx == 6)
        {
          //SendCmdToModem("ATE0\r");
          //while(!(strncmp(modemRxBuff+2, "OK", 2) == 0) && !cmdModemSet && !(cntModemRx == 6) && timeOffDelay){}
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          memset(consoleRxBuff, '\0', 255);
          counterPositive = 1654;
          counterNegative = 988;
          //printf("\r\nOK");
          //SendCmdToConsole("\r\n> ");
        } 
        /******************************************************************************
                            MODEM ON
        *******************************************************************************/
        if (strncmp(consoleRxBuff, "modem on", 8) == 0 && cmdConsoleSet && cntConsoleRx == 8)
        {
          //SendCmdToModem("ATE0\r");
          //while(!(strncmp(modemRxBuff+2, "OK", 2) == 0) && !cmdModemSet && !(cntModemRx == 6) && timeOffDelay){}
          startInitialize = 1;
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
          //printf("\r\nOK");
          //SendCmdToConsole("\r\n> ");
        } 
        if (strncmp(consoleRxBuff, "ATE0", 4) == 0 && cmdConsoleSet && cntConsoleRx == 4)
        {
          SendCmdToModem(consoleRxBuff);
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
        } 
        if (strncmp(consoleRxBuff, "AT", 2) == 0 && cmdConsoleSet && cntConsoleRx == 2)
        {
          SendCmdToModem(consoleRxBuff);
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
        } 
        if (cmdConsoleSet)
        {
          printf("\r\nUnknown command");
          cntConsoleRx = 0;
          cmdConsoleSet = 0;
          ClearBuffer(consoleRxBuff);
          SendCmdToConsole("\r\n> ");
        }
    }
/*******************************************************************************
                          AT command from MODEM
*******************************************************************************/
    /*if (cmdModemSet)
    {
        if (strncmp(modemRxBuff, "ATE0", 4) == 0 && cmdModemSet && cntModemRx == 13)
        {
          printf(modemRxBuff);
          cntModemRx = 0;
          cmdModemSet = 0;
          ClearBuffer(modemRxBuff);
          SendCmdToConsole("> ");
        } 
        if (strncmp(modemRxBuff+2, "OK", 2) == 0 && cmdModemSet && cntModemRx == 6)
        {
          printf(modemRxBuff);
          cntModemRx = 0;
          cmdModemSet = 0;
          ClearBuffer(modemRxBuff);
          SendCmdToConsole("> ");
        } 
        if (strncmp(modemRxBuff+2, "RING", 4) == 0 && cmdModemSet && cntModemRx == 8)
        {
          printf(modemRxBuff);
          cntModemRx = 0;
          cmdModemSet = 0;
          ClearBuffer(modemRxBuff);
        } 
        if (cmdModemSet)
        {
          //int i = strncmp(modemRxBuff+2, "OK", 2);
          //printf("\r\n\tCompare strings: %2i", i);
          printf("\r\ncntModemRx: %2u", cntModemRx);
          printf("\r\nUnknown command");
          cntModemRx = 0;
          cmdModemSet = 0;
          ClearBuffer(modemRxBuff);
          SendCmdToConsole("\r\n> ");
        }
    }*/
/*******************************************************************************
 *                           Initialization Modem                              *
 *                                                                             *
 *******************************************************************************/
    if (startInitialize)
    {
      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      timeOffDelay = 500;
      
      /* Send AT for checking modem's power */
      SendCmdToModem("AT\r");
      while(!cmdModemSet && timeOffDelay){}
      
      /* No response from modem. Turn on power of modem*/
      if (strncmp(modemRxBuff+2, "OK", 2) == 0)
      {
        printf("\r\nModem has already turned on");
      }
      else 
      {
        /* Вклчить питание с задержкой*/
        MODEMON;
        
        /* Wait 3 sec while modem starts scaning */
        timeOffDelay = 2500;
        while(timeOffDelay){}
        printf("\r\nPower is on\r\n");
        
        cntModemRx = 0;
        cmdModemSet = 0;
        memset(modemRxBuff, '\0', 255);
        
        timeOffDelay = 500;
        SendCmdToModem("ATE0\r");
        while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
        printf(modemRxBuff);
      }
      
      /* Ask for getting registration modem network in 1sec, max attempts 10 */
      for (int i = 0; i < 15; i++)
      {
        cntModemRx = 0;
        cmdModemSet = 0;
        memset(modemRxBuff, '\0', 255);
        int netReg = 0;
        timeOffDelay = 500;
        SendCmdToModem("AT+CREG?\r");
        while(!cmdModemSet && timeOffDelay){}
        printf(modemRxBuff);
        if (strncmp(modemRxBuff+2, "+CREG", 5) == 0)
        {
          sscanf(modemRxBuff + 11, "%d", &netReg);
          if (netReg == 1)
          {
            printf("\r\nModem is connected");
            break;
          }
          else 
          {
            printf("\r\nNumbers of attempts %2i", i);
          }
          timeOffDelay = 1000;
          while(timeOffDelay){}
        }
        else 
        {
          printf("\r\nNo response from modem");
          break;
        }
      }
      
      for (int i = 0; i < 60; i++)
      {
        cntModemRx = 0;
        cmdModemSet = 0;
        memset(modemRxBuff, '\0', 255);
        int netReg = 0;
        timeOffDelay = 500;
        SendCmdToModem("AT+CGATT?\r");
        while(!cmdModemSet && timeOffDelay){};
        printf(modemRxBuff);
        if (strncmp(modemRxBuff+2, "+CGATT", 6) == 0)
        {
          sscanf(modemRxBuff + 10, "%d", &netReg);
          if (netReg == 1)
          {
            printf("\r\nGPRS is attached");
            break;
          }
          else 
          {
            printf("\r\nNumbers of attempts %2i", i);
          }
          timeOffDelay = 1000;
          while(timeOffDelay){}
        }
        else 
        {
          printf("\r\nNo response from modem");
          break;
        }
      }
      
      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);     
      SendCmdToModem("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
      timeOffDelay = 500;
      while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
      printf(modemRxBuff);

 
      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      SendCmdToModem("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n");
      timeOffDelay = 500;
      while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
      printf(modemRxBuff);

      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      SendCmdToModem("AT+SAPBR=1,1\r\n");

      for (int i = 0; i < 60; i++)
      {
        cntModemRx = 0;
        cmdModemSet = 0;
        memset(modemRxBuff, '\0', 255);
        int netReg = 0;
        timeOffDelay = 500;
        SendCmdToModem("AT+SAPBR=2,1\r\n");
        while(!cmdModemSet && timeOffDelay){}
        printf(modemRxBuff);
        if (strncmp(modemRxBuff+2, "+SAPBR", 6) == 0)
        {
          sscanf(modemRxBuff + 12, "%d", &netReg);
          if (netReg == 1)
          {
            printf("\r\nConnection is set");
            break;
          }
          else 
          {
            printf("\r\nNumbers of attempts %2i", i);
          }
          timeOffDelay = 1000;
          while(timeOffDelay){}
        }
//        else 
//        {
//          printf("\r\nNo response from modem");
//          break;
//        }
      }            

      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      SendCmdToModem("AT+HTTPINIT\r\n");
      timeOffDelay = 500;
      while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
      printf(modemRxBuff);
      
      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      SendCmdToModem("AT+HTTPPARA=\"CID\",1\r\n");
      timeOffDelay = 500;
      while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
      printf(modemRxBuff);
      
      startInitialize = 0;
      readyModem = 1;
    }
    
    if (sendData)
    {
      printf("\n\r Positive: %5u\n\r", data1);
      printf("\n\r Negative: %5u\n\r", data2);
      
      sprintf(modemTxBuff, "AT+HTTPPARA=\"URL\",\"http://77.120.180.73/input.php?pol=%d&value=%d\"\r", data1, data2);
      
      timeOffDelay = 500;
      SendCmdToModem(modemTxBuff);   
      while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
      printf(modemRxBuff);
      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      
      timeOffDelay = 500;
      SendCmdToModem("AT+HTTPACTION=0\r");    
      while(!cmdModemSet && timeOffDelay){} //!(strncmp(modemRxBuff+2, "OK", 2) == 0) &&
      printf(modemRxBuff);
      cntModemRx = 0;
      cmdModemSet = 0;
      memset(modemRxBuff, '\0', 255);
      
      sendData = 0;
    }
  }
}

/******************************************************************************
                        SysTick_Handler
*******************************************************************************/
void SysTick_Handler(void)
{
  /* Счетчик для отсечта времени секунд */
  cntTimeMs += 1;
  WaitTickTime( &timeOffDelay );
  if (cntTimeMs == 1000)
  {
      /* Сбросить счетчик секунд */
      cntTimeMs = 0x00;
      
      /* Проверить наличие импульсов в каналах */
      /* если нету выключить питание (устройство) */
      if ((!counterNegative)&&(!counterPositive)&&(!sendData))
      {
        /*  */
        RTC_DateTypeDef RTC_CurrentDateStructure;
        RTC_GetDate(RTC_Format_BIN, &RTC_CurrentDateStructure);
        
        /*Получить время для установки следующего времени запуска */
        RTC_TimeTypeDef  RTC_gettingTimeStruct;
        RTC_GetTime(RTC_Format_BIN, &RTC_gettingTimeStruct);
        
        /* Установка следующего времени запуска устроуства по умолчанию, 30 сек */
        SetNextStartTime(RTC_gettingTimeStruct, 0, 0, 30); // Arguments: 0ro - current time, 1st - hours, 2nd - minutes, 3rd - seconds
        
        /* Выключаем питание устройства */
        PWROFF;
      }
      else
      {
          if (readyModem && !sendData)
          {
            data1 = counterPositive;
            data2 = counterNegative;
            sendData = 1;
            counterPositive = 0;
            counterNegative = 0;
          }
          if (!statusPwrModem)
          {
            statusPwrModem = 1; // Сброситься автоматически при выключении питания
            startInitialize = 1; // Разкоментить при работе в дежурном режиме
          }
      }
  }
}
/******************************************************************************
                        EXTI4_15_IRQHandler 
*******************************************************************************/
void EXTI4_15_IRQHandler(void)
{
  //Delay(5);
  
  /* Проверка канала прерывания негативных импульсов */
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    /* Toggle LED2 */
    counterNegative += 1;
    
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
  
  /* Проверка канала прерывания позитивных импульсов */
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    /* Toggle LED2 */
    counterPositive += 1;
    
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
  
}
/******************************************************************************
                        USART1_IRQHandler 
*******************************************************************************/
void USART1_IRQHandler(void)
{
   if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
   {
      /* Read one byte from the receive data register */
      uint8_t rxByte = USART_ReceiveData(USART1);
      
      if((rxByte == '\r') && (cntModemRx > 1))
      {
        modemRxBuff[cntModemRx] = rxByte;
        //cntModemRx += 1;
        cmdModemSet = 1;
      }
      else
      {
        modemRxBuff[cntModemRx] = rxByte;
        cntModemRx += 1;
      }
   }
}
/******************************************************************************
                        USART2_IRQHandler 
*******************************************************************************/
void USART2_IRQHandler(void)
{
   if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
   {
      /* Read one byte from the receive data register */
      uint8_t rxByte = USART_ReceiveData(USART2);
      
      if(rxByte == '\r')
      {
        consoleRxBuff[cntConsoleRx] = rxByte;
        cmdConsoleSet = 1;
      }
      else
      {
            // Эхо, чтобы не печатать вслепую
        USART_SendData(USART2, (uint8_t)rxByte);
            // Складываем символ в приёмный буфер
        consoleRxBuff[cntConsoleRx] = rxByte;
        cntConsoleRx += 1;
      }
   }
}
/******************************************************************************
                        RCC_Config
*******************************************************************************/
static void RCC_Config(void)
{
  /* Enable GPIO clock */
  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  /* Enable USART1 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  /* Enable USART2 Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
}
/******************************************************************************
                        GPIO_Config
*******************************************************************************/
static void GPIO_Config(void)
{
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | 
                        RCC_AHBPeriph_GPIOB | 
                        RCC_AHBPeriph_GPIOC, 
                        ENABLE);
 
  /* USART1 Pins configuration */
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);    
  
  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  /* USART2 Pins configuration */
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);    
  
  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* PWR, LED Pin configuration */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //(_OUT, _AF, _AN)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //(_2MHz, _10MHz, 40MHz)
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //(_NOPULL, _UP)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_15; //(_0 ... _15)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //(_PP - push/pull, _OD - open drain)
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*MODEM ON Pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*STOP Pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //(_0 ... _15)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //(_OUT, _AF, _AN)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //(_2MHz, _10MHz, 40MHz)
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //(_NOPULL, _UP)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //(_PP - push/pull, _OD - open drain)
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure PB5 and PB8 pins as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
}
/******************************************************************************
                        RTC_Config
*******************************************************************************/
static void RTC_Config(void)
{
    RTC_InitTypeDef RTC_InitStructure;
    
    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);
    
    /* Reset RTC Domain */
    //RCC_BackupResetCmd(ENABLE);
    //RCC_BackupResetCmd(DISABLE);
    
    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);
    
    /* Wait till LSE is ready */  
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}
    
    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    
    /* Configure the RTC data register and RTC prescaler */
    /* ck_spre(1Hz) = RTCCLK(LSI) /(AsynchPrediv + 1)*(SynchPrediv + 1)*/
    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv  = 0xFF;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);

    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
    
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();   
}
/******************************************************************************
                        RTC_AlarmConfig
*******************************************************************************/
static void RTC_AlarmConfig(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* RTC Alarm A Interrupt Configuration */
  /* EXTI configuration */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel            = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority    = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Output AlarmA Pin configuration */
  RTC_OutputConfig(RTC_Output_AlarmA, RTC_OutputPolarity_High);
  RTC_OutputTypeConfig(RTC_OutputType_PushPull);

  /* Enable AlarmA interrupt */
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
  RTC_ClearITPendingBit(RTC_IT_ALRA);
} 
/******************************************************************************
                        RTC_IRQHandler
*******************************************************************************/
void RTC_IRQHandler(void)
{
  /* Check on the AlarmA flag and on the number of interrupts per Second (60*8) */
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET) 
  { 
    /* ALARM is enabled */
    ALARM_Occured = 1;
    //LEDXOR;
    
    /* Clear RTC AlarmA Flags */
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);
  }  
}
/******************************************************************************
                        SetCurrentTime
*******************************************************************************/
static void SetCurrentTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    RTC_TimeTypeDef  RTC_TimeStruct;
    /* Set the default time to 00h 00mn 00s or shif to */
    RTC_TimeStruct.RTC_H12     = 0x00;
    RTC_TimeStruct.RTC_Hours   = 0x00 + hours;
    RTC_TimeStruct.RTC_Minutes = 0x00 + minutes;
    RTC_TimeStruct.RTC_Seconds = 0x00 + seconds;
    
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
}
/******************************************************************************
                        SetCurrentDate
*******************************************************************************/
static void SetCurrentDate(uint8_t date, uint8_t month, uint8_t year)
{
    RTC_DateTypeDef RTC_DateStructure;
    
    //RTC_DateStructInit(&RTC_DateStructure);
    /* Set the default date to 01d 01m 00y or shif to */
    RTC_DateStructure.RTC_Date = date;
    RTC_DateStructure.RTC_Month = month;
    //RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Saturday;
    RTC_DateStructure.RTC_Year = year;

    RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
}
/******************************************************************************
                        SetNextStartTime
*******************************************************************************/
static void SetNextStartTime(RTC_TimeTypeDef  RTC_currentTimeStruct, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  /* Настройка будильника */
  RTC_AlarmTypeDef RTC_AlarmStructure;
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
  RTC_AlarmStructInit(&RTC_AlarmStructure);
    
  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12      = 0x00;
  /* Проверить диапазон задания значения СЕКУНД, исправить */
  //seconds = seconds &  
  if ((RTC_currentTimeStruct.RTC_Seconds + seconds) > 59)
    {
      RTC_currentTimeStruct.RTC_Seconds = (RTC_currentTimeStruct.RTC_Seconds + seconds) - 60;
    }
  else RTC_currentTimeStruct.RTC_Seconds = RTC_currentTimeStruct.RTC_Seconds + seconds;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds  = RTC_currentTimeStruct.RTC_Seconds;
  
  /* Проверить диапазон задания значения МИНУТ, исправить */
  if ((RTC_currentTimeStruct.RTC_Minutes + minutes) > 59)
    {
      RTC_currentTimeStruct.RTC_Minutes = (RTC_currentTimeStruct.RTC_Minutes + minutes) - 60;
    }
  else RTC_currentTimeStruct.RTC_Minutes = RTC_currentTimeStruct.RTC_Minutes + minutes;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes  = RTC_currentTimeStruct.RTC_Minutes;
  
  /* Проверить диапазон задания значения ЧАСОВ, исправить */
  if ((RTC_currentTimeStruct.RTC_Hours + hours) > 23)
    {
      RTC_currentTimeStruct.RTC_Hours = (RTC_currentTimeStruct.RTC_Hours + hours) - 24;
    }
  else RTC_currentTimeStruct.RTC_Hours = RTC_currentTimeStruct.RTC_Hours + hours;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours    = RTC_currentTimeStruct.RTC_Hours;
  
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel    = RTC_AlarmDateWeekDaySel_Date;
    //RTC_AlarmStructure.RTC_AlarmDateWeekDay     = RTC_Weekday_Monday;    
  RTC_AlarmStructure.RTC_AlarmMask              = RTC_AlarmMask_Minutes | 
                                                  RTC_AlarmMask_Hours | 
                                                  RTC_AlarmMask_DateWeekDay;
    
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
  
  /* Enable AlarmA */
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}
/******************************************************************************
                        USART1 Configurating
*******************************************************************************/
static void USART1_Config(void)
 {
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   USART_Cmd(USART1, DISABLE);
  /* USARTx configuration */
  /* USARTx configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - Stop Bit = 1 Stop Bit
  - Parity = No Parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled*/
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  /* NVIC configuration */
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable USART */
  USART_Cmd(USART1, ENABLE); 
 }
/******************************************************************************
                          USART2 Configurating
*******************************************************************************/
static void USART2_Config(void)
 {
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   USART_Cmd(USART2, DISABLE);
  /* USARTx configuration */
  /* USARTx configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - Stop Bit = 1 Stop Bit
  - Parity = No Parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled*/
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  /* NVIC configuration */
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable USART */
  USART_Cmd(USART2, ENABLE); 
 }
/******************************************************************************
                          EXTI4_15 Configurating
*******************************************************************************/
static void EXTI4_15_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Connect EXTI8 Line to PB5 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
  
  /* Connect EXTI13 Line to PB8 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);
  
  /* Configure EXTI5 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;  
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Configure EXTI8 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI4_15 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
   
/******************************************************************************
                          DELAY - 1
*******************************************************************************/
void Delay( unsigned int Val) 
{
  for( ; Val != 0; Val--) 
  {
    __NOP();
  }
}
/******************************************************************************
                          DELAY - 2
*******************************************************************************/
static void WaitTickTime(uint32_t *waitTime)
{
  if (*waitTime > 0) { *waitTime -= 1; }
}
/******************************************************************************
                          SendCmdToModem
*******************************************************************************/
static void SendCmdToModem(char *buffRx)
{
  //uint8_t buffRxSize = strlen(buffRx);
  uint8_t byteCounter = 0;
  while(buffRx[byteCounter])
  {
    USART_SendData(USART1, (uint8_t) buffRx[byteCounter++]);
    /* Loop until transmit data register is empty */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
  }
}
/******************************************************************************
                          SendCmdToConsole
*******************************************************************************/
static void SendCmdToConsole(char *buffRx)
{
  //uint8_t buffRxSize = strlen(buffRx);
  uint8_t byteCounter = 0;
  while(buffRx[byteCounter])
  {
    USART_SendData(USART2, (uint8_t)buffRx[byteCounter++]);
    /* Loop until transmit data register is empty */
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
  }
}
static void ClearBuffer(char* buff)
{
  for (int i = 0; i < 256; i++)
  {
    buff[i] = '\0';
  }
}
/******************************************************************************
                          PUTCHAR_PROTOTYPE
*******************************************************************************/
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {}
  return ch;
}
void put_rc(FRESULT rc)
{
	char *str =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (i = 0; i != rc && *str; i++) {
		while (*str++) ;
	}
	printf("rc=%u FR_%s\n", (UINT)rc, str);;
}
