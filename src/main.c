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
                            Delay(9500000);\
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
char cmdRxBuff[BUFFSIZE];
char cmdTxBuff[BUFFSIZE];

char userRxBuff[BUFFSIZE];
char userTxBuff[BUFFSIZE];

__IO uint8_t cntCmdRx = 0;
__IO uint8_t cntUserRx = 0;

//__IO uint8_t SD_RdWrBuff[SDBUFFSIZE];     
//__IO uint8_t SD_RdBuff[SDBUFFSIZE];

//__IO uint8_t CMD_RdWrBuff[CMDBUFFSIZE];
//__IO uint8_t CMD_RdBuff[CMDBUFFSIZE];

__IO uint8_t ALARM_Occured = 0;

///char Line[256];				/* Console input buffer */
//BYTE Buff[4096]; //__attribute__ ((aligned (4))) ;	/* Working buffer */
//char Line_Rx[256];

//RTCTIME rtc;
//FATFS FatFs;				/* File system object for each logical drive */
FIL file;				/* File objects */
//DIR dir;					/* Directory object */
__IO uint32_t cntTimeMs;
__IO uint32_t cntTimeDelay;

__IO uint32_t counterPositive = 0;
__IO uint32_t counterNegative = 0;
volatile unsigned char flag =0;
//------------------------------------------------------------------------------
void Delay(unsigned int Val);
void InitIRQ(void);

static void RCC_Config(void);
static void USART1_Config(void);
static void USART2_Config(void);
static void RTC_Config(void);
static void RTC_DateConfig(void);
static void GPIO_Config(void);
static void RTC_AlarmConfig(void);
static void EXTI4_15_Config(void);

static void SendCmdToModem(char *buffRx);
static void SetNextStartTime(void);
static void SetCurrentTime(void);

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

  //NVIC_EnableIRQ (USART1_IRQn);
  //NVIC_EnableIRQ (EXTI9_5_IRQn);

  /*if (rtc_initialize()) 
    {
      rtc_gettime(&rtc);
    } 
  else 
  {
    UART1_Tx_Str("RTC is not available.\n");
  }
  // смонтировать диск
 

  //FATFS *fs;
  result = f_mount(&FatFs, "", 1);
  if (result != FR_OK)
    {
      put_rc(result);
    }
  Delay(950000);
  InitIRQ();
  NVIC_EnableIRQ (EXTI9_5_IRQn);
  // отключаем команду
  sprintf(Line, "ATE0\r\n");
  UART1_Tx_Str(Line);
    Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  sprintf(Line, "AT+SAPBR=1,1\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
   Delay(950000);
  Delay(950000);
   Delay(950000);
  Delay(950000);
  sprintf(Line, "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
  sprintf(Line, "AT+SAPBR=3,1,\"APN\",\"kyivstar.net\"\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
  sprintf(Line, "AT+SAPBR=3,1,\"USER\",\"igprs\"\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
  sprintf(Line, "AT+SAPBR=3,1,\"PWD\",\"internet\"\r\n");
  UART1_Tx_Str(Line);
  sprintf(Line, "AT+SAPBR=3,1,\"RATE\",3\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);

  sprintf(Line, "AT+HTTPINIT\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
  sprintf(Line, "AT+HTTPPARA=\"CID\",1\r\n");
  UART1_Tx_Str(Line);
  Delay(950000);
  Delay(950000);
  __enable_irq ();*/
 
  /*FRESULT result;
  // смонтировать диск
  FATFS FATFS_Obj;
  result = f_mount(&FATFS_Obj, "0", 1);
  if (result != FR_OK)
    {
      put_rc(result);
    }
  else 
  
  printf("Write!!!!!!!!!!!!!!!!");
  result = f_open(&file, "config.txt", FA_OPEN_EXISTING | FA_READ);
        if (result == FR_OK) 
          {
            //UART1_Tx_Str(buff);
            int str_lenth = strlen(SD_RdWrBuff);
            //f_lseek(&file, f_size(&file));
            //f_lseek(&file, ofs_crs);
            f_read(&file, buffRdWr, str_lenth, &nWritten);
            f_close(&file);
          }
        else {
          put_rc(result);
         }
    
  __enable_irq ();*/
  while(1) 
  {
          //uint32_t subSec = RTC_GetSubSecond();
      //printf("%d\r\n", subSec);
    //uint32_t timeLoopStop = 0;
    //cntTimeDelay = 2000;
    //while ((strncmp(cmdRxBuff, "OK", 2)))
      //{ if (cntTimeDelay == 0){break;} }
    //printf("Good job!");
    //memset(cmdRxBuff, 0, BUFFSIZE);
    //cntCmdRx = 0x00;
    //ptr2 = cmd; 
    //if (num == 0)
    //{
    if ((strncmp(userRxBuff, "st\r\n", 4)) == 0)
    {
      LEDXOR;
      SetCurrentTime();
      cntUserRx = 0;
      memset(userRxBuff,'\0',4);
      //SysTick_Config(SystemCoreClock / 1000);
    }
    
    /*switch (userRxBuff[0])
    {
    case time:
        switch (userRxBuff[1])
         {
                  // command set time
         case set: 
                   RTC_Config();
                   RTC_AlarmConfig();
                   break;
                   // Command get time
         case get: RTC_TimeTypeDef  RTC_TimeStruct;
                   RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
                   sprintf(
                           userTxBuff, 
                           "\r\n%2u:%02u:%02u\r\n",  
                           RTC_TimeStruct.RTC_Hours, 
                           RTC_TimeStruct.RTC_Minutes, 
                           RTC_TimeStruct.RTC_Seconds
                           );
                   printf(userTxBuff);
                   memset(userTxBuff,'\0',10);
                   break;
         }
    }*/
  }
}
//-----------------------------------------------------------------------------
void InitIRQ(void)
{
  //RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  //External interrupt
  //Interrupt INT1 for PORTB.
  //AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;     
  EXTI->RTSR |= EXTI_RTSR_TR6;
  EXTI->IMR |= EXTI_IMR_MR6; 
  
  //External interrupt
  //Interrupt INT2 for PORTB.
  //AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PB;     
  EXTI->FTSR |= EXTI_FTSR_TR8;
  EXTI->IMR |= EXTI_IMR_MR8;
  
  //Timer interrupt
  
} 
/******************************************************************************
                        SysTick_Handler
*******************************************************************************/
void SysTick_Handler(void)
{
  /* Счетчик для отсечта времени секунд */
  cntTimeMs += 1;

  if (cntTimeMs == 1000)
  {
      /* Сбросить счетчик секунд */
      cntTimeMs = 0x00;
      
      /* Проверить наличие импульсов в каналах */
      /* если нету выключить питание (устройство) */
      if ((!counterNegative)&&(!counterPositive))
      {
        /*Получить время для отправки в консоль перед выкл (не обязательно)*/
        RTC_TimeTypeDef  RTC_TimeStruct;
        RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
        printf("\n\r%02d %02u:%02u:%02d\n\r", RTC_TimeStruct.RTC_H12, 
                                              RTC_TimeStruct.RTC_Hours, 
                                              RTC_TimeStruct.RTC_Minutes, 
                                              RTC_TimeStruct.RTC_Seconds
                                              );
        
        /* Установка следующего времени запуска устроуства по умолчанию, 30 сек */
        SetNextStartTime();
        
        /* Выключаем питание устройства */
        PWROFF;
      }
      else
      {
        if (counterPositive)
        {
          printf("\n\r Positive: %5u\n\r", counterPositive);
          counterPositive = 0;
        }
        if (counterNegative)
        {
          printf("\n\rNegative: %5u\n\r", counterNegative);
          counterNegative = 0;
        }
      }
  }
  /*else
    {
      //создаем файл с текущей датой и время создания 010720151522.txt
      if (cnt_puls_pos)
      {
        char buff[255] = {0};
        result = f_open(&file, "readme.txt", FA_OPEN_EXISTING | FA_WRITE);
        if (result == FR_OK) 
          {
            rtc_gettime(&rtc);
            sprintf(buff, ".%02u%02u%u%02u%02u%02uP%d", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec, cnt_puls_pos);
            //UART1_Tx_Str(buff);
            int str_lenth = strlen(buff);
            f_lseek(&file, f_size(&file));
            //f_lseek(&file, ofs_crs);
            f_write(&file, buff, str_lenth, &nWritten);
            f_close(&file);
            sprintf(buff, "AT+HTTPPARA=\"URL\",\"http://77.120.180.73/input.php?pol=P&value=%d\"\r\n", cnt_puls_pos);
            UART1_Tx_Str(buff);
            Delay(950000);
            Delay(950000);
            sprintf(buff, "AT+HTTPACTION=0\r\n");
            UART1_Tx_Str(buff);
            Delay(950000);
            Delay(950000);
            Delay(950000);
            Delay(950000);
            Delay(950000);
          }
        else {put_rc(result);}
        cnt_puls_pos = 0;
      }
      if (cnt_puls_neg)
      { 
        char buff[255] = {0};
        result = f_open(&file, "readme.txt", FA_OPEN_EXISTING | FA_WRITE);
        if (result == FR_OK) 
          {
            rtc_gettime(&rtc);
            sprintf(buff, ".%02u%02u%u%02u%02u%02uN%d", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec, cnt_puls_neg);
            UART1_Tx_Str(buff);
            int str_lenth = strlen(buff);
            ofs_crs = f_tell(&file);
            f_lseek(&file, f_size(&file));
            f_write(&file, buff, str_lenth, &nWritten);
            //ofs_crs += str_lenth;
            f_close(&file);
          }
        else {put_rc(result);}
        cnt_puls_neg = 0;
      }
      cnt_puls_pos = 0;
      cnt_puls_neg = 0;
    }
  }
  disk_timerproc();*/
}

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
      //LEDON;
      /* Read one byte from the receive data register */
      cmdRxBuff[cntCmdRx++] = USART_ReceiveData(USART1);
      //if(cntCmdRx == BUFFSIZE)
      //{
        /* Disable the EVAL_COM1 Receive interrupt */
        //cntCmdRx = 0x00;
      //}
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
      userRxBuff[cntUserRx++] = USART_ReceiveData(USART2);
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
 
  /* USART1 Pins configuration ************************************************/
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
  
  /* USART2 Pins configuration ************************************************/
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
  
  /* PWR, LED Pin configuration ****************************************/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //(_OUT, _AF, _AN)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //(_2MHz, _10MHz, 40MHz)
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //(_NOPULL, _UP)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_15; //(_0 ... _15)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //(_PP - push/pull, _OD - open drain)
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /*MODEM ON Pin configuration ************************************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /*STOP Pin configuration ************************************************/
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
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_12;
    RTC_Init(&RTC_InitStructure);
    
    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
    
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();   
}
/******************************************************************************
                        RTC_DateConfig
*******************************************************************************/
static void RTC_DateConfig(void)
{
  RTC_DateTypeDef RTC_DateStructure;
  RTC_DateStructInit(&RTC_DateStructure);
  RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
}
/******************************************************************************
                        RTC_AlarmConfig
*******************************************************************************/
static void RTC_AlarmConfig(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*RTC_AlarmStructInit(&RTC_AlarmStructure);
  
  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12      = RTC_H12_AM;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds  = 10;
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel    = RTC_AlarmDateWeekDaySel_Date;
  //RTC_AlarmStructure.RTC_AlarmDateWeekDay       = RTC_Weekday_Monday;    
  RTC_AlarmStructure.RTC_AlarmMask              = RTC_AlarmMask_Minutes | 
                                                  RTC_AlarmMask_Hours | 
                                                  RTC_AlarmMask_DateWeekDay;
  
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);*/
  
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
                        SetNextStartTime
*******************************************************************************/
static void SetCurrentTime(void)
{
    RTC_TimeTypeDef  RTC_TimeStruct;
  
    /* Set the time to 00h 00mn 00s AM */
    RTC_TimeStruct.RTC_H12     = RTC_H12_AM;
    RTC_TimeStruct.RTC_Hours   = 0x00;
    RTC_TimeStruct.RTC_Minutes = 0x00;
    RTC_TimeStruct.RTC_Seconds = 0x00;
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
}
/******************************************************************************
                        SetNextStartTime
*******************************************************************************/
static void SetNextStartTime(void)
{
  /*Получаем текущее время и устанавливаем будильник на 10 сек вперед*/
  RTC_TimeTypeDef  RTC_currentTimeStruct;
  RTC_GetTime(RTC_Format_BIN, &RTC_currentTimeStruct);
  
  /* Настройка будильника на 10 сек вперед*/
  RTC_AlarmTypeDef RTC_AlarmStructure;
  RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
  RTC_AlarmStructInit(&RTC_AlarmStructure);
    
  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12      = RTC_currentTimeStruct.RTC_H12;
  if ((RTC_currentTimeStruct.RTC_Seconds + 10) > 59)
    {
      RTC_currentTimeStruct.RTC_Seconds = (RTC_currentTimeStruct.RTC_Seconds + 10) - 60;
    }
  else RTC_currentTimeStruct.RTC_Seconds = RTC_currentTimeStruct.RTC_Seconds + 10;

  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds  = RTC_currentTimeStruct.RTC_Seconds;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes  = RTC_currentTimeStruct.RTC_Minutes;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours    = RTC_currentTimeStruct.RTC_Hours;
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel    = RTC_AlarmDateWeekDaySel_Date;
    //RTC_AlarmStructure.RTC_AlarmDateWeekDay     = RTC_Weekday_Monday;    
  RTC_AlarmStructure.RTC_AlarmMask              = RTC_AlarmMask_Minutes | 
                                                  RTC_AlarmMask_Hours | 
                                                  RTC_AlarmMask_DateWeekDay;
    
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
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
  
  /* Config Output Pin */
  RTC_OutputConfig(RTC_Output_AlarmA, RTC_OutputPolarity_High);
  RTC_OutputTypeConfig(RTC_OutputType_PushPull);

  /* Enable AlarmA interrupt */
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_ClearITPendingBit(RTC_IT_ALRA);
  EXTI_ClearITPendingBit(EXTI_Line17);
  
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
  /* USARTx configuration ----------------------------------------------------*/
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
  /* USARTx configuration ----------------------------------------------------*/
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
                          DELAY
*******************************************************************************/
void Delay( unsigned int Val) 
{
  for( ; Val != 0; Val--) 
  {
    __NOP();
  }
}
/******************************************************************************
                          SendCmdToModem
*******************************************************************************/
static void SendCmdToModem(char *buffRx)
{
  uint8_t buffRxSize = strlen(buffRx);
  uint8_t byteNumber = 0;
  while(buffRxSize--)
  {
    USART_SendData(USART1, (uint8_t) buffRx[byteNumber++]);
    /* Loop until transmit data register is empty */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
  }
}
/*uint8_t ToTimeProc(uint32_t timeCount){
  timeCount--;
  if (timeCount == 0)
    return 0;
  else return 1;
}*/
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
