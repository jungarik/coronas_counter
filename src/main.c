#include <string.h>
#include <stdio.h>
#include "stm32f0xx.h"
#include "system_stm32f0xx.h"
//#include "stm32f0xx_conf.h"
//#include "diskio.h"
#include "ff.h"
//#include "ffconf.h"
//#include "sd.c"
//#include "STM32F100.h"
//#include "xprintf.h"

#define F_CPU           48000000UL
#define TimerTick	F_CPU/1000-1 // 1 kHz
#define PowerOFF        GPIOB -> BSRR = GPIO_BSRR_BR_4;

/*#define SPI_SD SPI1
#define spi_cs_low() do { GPIOA -> BRR = GPIO_Pin_4;} while (0)
#define spi_cs_high() do { GPIOA -> BSRR = GPIO_Pin_4;} while (0)*/
//#define spi_cs_low() GPIOA -> BSRR = GPIO_BSRR_BR_4, spi_txrx(0xFF);
//#define spi_cs_high() GPIOA -> BSRR = GPIO_BSRR_BS_4, spi_txrx(0xFF);

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
UINT nRead, nWritten;
//DWORD ofs_crs = 0;
unsigned char str_lenth = 0;
//unsigned char sh;
char buff[255] = "123456";

__IO uint8_t ALARM_Occured = 0;


#if _USE_LFN
char Lfname[512];

#endif

char Line[256];				/* Console input buffer */
//BYTE Buff[4096]; //__attribute__ ((aligned (4))) ;	/* Working buffer */
char Line_Rx[256];

//RTCTIME rtc;
//FATFS FatFs;				/* File system object for each logical drive */
FIL file;				/* File objects */
//DIR dir;					/* Directory object */
DWORD count_ms;
char cmd[256];
int num = 0;
/*enum sd_speed 
    { 
      SD_SPEED_INVALID, 
      SD_SPEED_400KHZ, 
      SD_SPEED_25MHZ 
    };*/
//volatile UINT Timer;

uint32_t cnt_puls_pos = 0;
uint32_t cnt_puls_neg = 0;
volatile unsigned char flag =0;
//------------------------------------------------------------------------------
void InitSysClock(void);
void InitGPIO(void);
void Delay(unsigned int Val);
void InitIRQ(void);
void InitSysTick(void);
static void RCC_Config(void);
static void USART1_Config(void);
static void RTC_Config(void);
static void RTC_DateConfig(void);
static void GPIO_Config(void);
static void RTC_AlarmConfig(void);
void RTC_IRQHandler(void);
//-------------------------------------------------------------------
/*void spi_set_speed(enum sd_speed speed);
void spi_init(void);
uint8_t spi_txrx(uint8_t data);
void sd_nec(void);
void sd_cmd(uint8_t cmd, uint32_t arg);
uint8_t sd_get_r1(void);
uint8_t crc7_one(uint8_t t, uint8_t data);*/



//void RTC_IRQHandler(void)
//static void USART_SendString();
void InitIWDG(void);
void put_rc(FRESULT rc);
int StrToInt(char *ptr);



 
//------------------------------------------------------------------------------
 
int main( void) 
{
  
  //char *ptr2;
  //char *ptr;
  SystemInit();
  //InitSysClock();
  InitGPIO();
  GPIO_Config();
  //GPIOB -> BSRR = GPIO_BSRR_BS_4;
  InitSysTick();
  RCC_Config();
  RTC_Config();
  RTC_DateConfig();
  USART1_Config();
  USART_Cmd(USART1, ENABLE);
  RTC_AlarmConfig();
    Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
      Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  Delay(950000);
  //spi_init();

  //InitIRQ();
  //InitUART();
  //NVIC_EnableIRQ (USART1_IRQn);
  //NVIC_EnableIRQ (EXTI9_5_IRQn);
  //rtc_initialize();
  //rtc_gettime(&rtc);
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
  /*Delay(9000000);
  Delay(9000000);
  Delay(9000000);
  GPIOA->BSRR = GPIO_BSRR_BS_10;
  Delay(9000000);
  Delay(9000000);
  Delay(9000000);
  GPIOA->BSRR = GPIO_BSRR_BR_10;*/  
  FRESULT result;

  // смонтировать диск
  FATFS FATFS_Obj;
  //result = f_write(&file, buff, str_lenth, &nWritten);

  result = f_mount(&FATFS_Obj, "0", 1);
  if (result != FR_OK)
  {
    put_rc(result);
  }
  
  printf("Write!!!!!!!!!!!!!!!!");
  result = f_open(&file, "readme.txt", FA_OPEN_EXISTING | FA_WRITE);
        if (result == FR_OK) 
          {
            //UART1_Tx_Str(buff);
            int str_lenth = strlen(buff);
            //f_lseek(&file, f_size(&file));
            //f_lseek(&file, ofs_crs);
            f_write(&file, buff, str_lenth, &nWritten);
            f_close(&file);
          }
        else {put_rc(result);}
    
  __enable_irq ();
  while(1) 
  {
    /*ptr2 = cmd; 
    if (num == 0)
    {
    switch (cmd[0])
    {
    case 't':
        switch (cmd[1])
         {
                  // command set time
         case 's': 
                                rtc.mday = atoi(&cmd[3]);
				rtc.month = atoi(&cmd[6]);
				rtc.year = atoi(&cmd[9]);
                                rtc.hour = atoi(&cmd[14]); 
                                rtc.min = atoi(&cmd[17]);
				rtc.sec = 00;
                                rtc_settime(&rtc);
                   memset(cmd,'\0',10);
                   break;
                   // Command get time
         case 'g': rtc_gettime(&rtc);
                   sprintf(Line, "%u/%u/%u %2u:%02u:%02u \0", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec);
                   UART1_Tx_Str(Line);
                   memset(cmd,'\0',10);
                   break;
         }
        break;
    case 'w':
      switch (cmd[1])
      {
        case 'u': sleep = 0;
                  UART1_Tx_Str("cmd_wu"); 
                  memset(cmd,'\0',10); 
                  break;
        case 'd': UART1_Tx_Str("cmd_wd"); 
                  memset(cmd,'\0',10); 
                  sleep = 1;
                  break;
        } break;
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

void InitSysTick(void)
{
  SysTick->LOAD = TimerTick;
  SysTick->VAL = TimerTick;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk | 
                  SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
  count_ms += 1;
  if (count_ms == 5000)
  {
      RTC_TimeTypeDef  RTC_TimeStruct;
      //GPIO_InitTypeDef GPIO_InitStructure;
      GPIOB -> ODR ^= GPIO_ODR_15;
      RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
      printf("\n\r%02u:%02u:%02d\n\r", RTC_TimeStruct.RTC_Hours, RTC_TimeStruct.RTC_Minutes, RTC_TimeStruct.RTC_Seconds);
      //sprintf(Line, "%u/%u/%u %2u:%02u:%02u \0", RTC_TimeStruct.RTC_Hours, RTC_TimeStruct.RTC_Minutes, RTC_TimeStruct.RTC_Seconds);
      /* Loop until the end of transmission */
      /* The software must wait until TC=1. The TC flag remains cleared during all data
         transfers and it is set by hardware at the last frame’s end of transmission*/
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
      count_ms = 0x00;
      
     //owerOFF; // Выключить питание устройства.
      
  }
  //disk_timerproc();
  /*if (count_ms == 1000)
  {
    count_ms = 0;
    if ((!cnt_puls_neg)&&(!cnt_puls_pos)&&(!(GPIOB->IDR&GPIO_IDR_12)))
    {
      IWDG->KR |= 0x5555;       //enable access to the IWDG_PR and IWDG_RLR registers
      IWDG->PR |= 0x6;          //divider /128
      IWDG->RLR |=0xFFF;        //watchdog counter reload value 13000ms
      IWDG->KR |= 0xCCCC;       //starts the watchdog 
      // Clear Wake-up flag 
      PWR->CR |= PWR_CR_CWUF;
      // Select STANDBY mode 
      PWR->CR |= PWR_CR_PDDS;
      // Set SLEEPDEEP bit of Cortex System Control Register 
      SCB->SCR |= SCB_SCR_SLEEPDEEP;
      __WFI();
  } 
    else
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

/*DWORD get_fattime (void)
{
        RTCTIME rtc;
	// Get local time 
	if (!rtc_gettime(&rtc)) return 0;
	// Pack date and time into a DWORD variable 
	return	  ((DWORD)(rtc.year - 1980) << 25)
		| ((DWORD)rtc.month << 21)
		| ((DWORD)rtc.mday << 16)
		| ((DWORD)rtc.hour << 11)
		| ((DWORD)rtc.min << 5)
		| ((DWORD)rtc.sec >> 1);
}*/
void EXTI9_5_IRQHandler(void)
{
  Delay(5);
  if ((EXTI->PR)&EXTI_PR_PR8)
  {
    cnt_puls_pos += 1;
  }
  else cnt_puls_neg += 1;
  EXTI->PR |= EXTI_PR_PR8;
  EXTI->PR |= EXTI_PR_PR6;
}

void USART1_IRQHandler(void)
{
 if (USART1->ISR & USART_ISR_RXNE)
	{
	Line[num]= USART1->RDR;
        if (cmd[num]=='\0'){
          num = 0;
        }
        else {num += 1;}
	}
}

void InitGPIO( void) 
{
  // Enable PORTB Periph clock  
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  
  //For Enter to Debug Mode PB12 PIN25 Input, resistor Pull-Down
  /*GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12); // reset config PortB8
  GPIOB->CRH |= GPIO_CRH_CNF12_1;
  GPIOB->BSRR = GPIO_BSRR_BS12;
  //For Ext.Interrupt PB8 Input PIN45, resistor Pull-Down
  GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); // reset config PortB8
  GPIOB->CRH |= GPIO_CRH_CNF8_1;
  GPIOB->BSRR = GPIO_BSRR_BS8;*/
  
  /* Config PB7 PIN43 For Power Input Part*/
  /*GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7); // reset config PortB8
  GPIOB->CRL |= GPIO_CRL_MODE7_1;
  //GPIOB->CRL = GPIO_CRL_CNF7_1;
  GPIOB->BSRR = GPIO_BSRR_BS7;*/
  
  /*For Ext.Interrupt PB6 PIN42 Input, resistor Pull-Down*/
  /*GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
  GPIOB->CRL |= GPIO_CRL_CNF6_1;
  //GPIOB->BSRR = GPIO_BSRR_BS6;*/
  
  /* Config PortB 0*/
  /*GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // reset config PortB0
  GPIOB->CRL |= GPIO_CRL_MODE0_1;
  GPIOB->BSRR = GPIO_BSRR_BS0;)*/
  
  // Enable PORTC Periph clock  
  //RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;*/
  // For GREEN LED PC10 Output
  GPIOB->MODER &= ~GPIO_MODER_MODER15;
  GPIOB->MODER |= GPIO_MODER_MODER15_0;
  GPIOA->MODER &= ~GPIO_MODER_MODER10;
  GPIOA->MODER |= GPIO_MODER_MODER10_0;
  // Enable PORTA Periph clock
  /*RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;;
  
  GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
  GPIOA->CRH |= GPIO_CRH_CNF8_1;         //PA9 alternate function output Push-pull
  GPIOA->CRH |= GPIO_CRH_MODE8_0;       //PA9 output mode, max speed 10 MHz
  
  //For UART. PA9 Output, PA10 Input
  GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
  GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
  GPIOA->CRH |= GPIO_CRH_CNF9_1;         //PA9 alternate function output Push-pull
  GPIOA->CRH |= GPIO_CRH_MODE9_0;       //PA9 output mode, max speed 10 MHz
  GPIOA->CRH |= GPIO_CRH_CNF10_1 ;        //PA10 input with pull-up / pull-down */
  //For RF sleep mode PA12 Output, Push-Pull
  /*GPIOA->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
  GPIOA->CRH |= GPIO_CRH_MODE12_1;
  GPIOA->BSRR = GPIO_BSRR_BS12;*/
  //For RF sleep mode PA8 Output, Push-Pull
  /*GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
  GPIOA->CRH |= GPIO_CRH_MODE8_1;
  GPIOA->BSRR = GPIO_BSRR_BS8;*/
  //For SPI mode PA4, PA5, PA7 - output, PA6 - input
  /*GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5 |
                  GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7 ); //Reset PA4, PA5, PA7, PA6
  GPIOA->CRL |= GPIO_CRL_MODE4_0 | GPIO_CRL_CNF4_1      //PA4 alternate function output Push-pull SPI - CS Out
              | GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_1      //PA5 alternate function output Push-pull SPI - SCLK OUT
              | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1      //PA7 alternate function output Push-pull SPI - MOSI OUT
              | GPIO_CRL_CNF6_1;                        //PA6 PA10 input with pull-up / pull-down SPI - MISO IN*/
  
  
}

static void RCC_Config(void)
{
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE);
  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  /* Enable USART1 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
}

static void GPIO_Config(void)
{
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* USART1 Pins configuration ************************************************/
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);    
  
  /* Configure pins as AF pushpull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* RTC_Out Pin configuration ************************************************/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //(_OUT, _AF, _AN)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; //(_2MHz, _10MHz, 40MHz)
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //(_NOPULL, _UP)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //(_0 ... _15)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //(_PP - push/pull, _OD - open drain)
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_PinAFConfig(GPIOC, GPIO_PinSource13, GPIO_AF_0);*/
  
}
void InitSysClock(void)
{
   RCC->CR |= RCC_CR_HSEON;                       // Включить генератор HSE.
   while (!(RCC->CR & RCC_CR_HSERDY)) {};       // Ожидание готовности HSE.
     __NOP();
   FLASH->ACR |= FLASH_ACR_PRFTBE;              // Enable Prefetch Buffer.
   FLASH->ACR |= FLASH_ACR_LATENCY;           // Если 48< SystemCoreClock <= 72, пропускать 2 такта.*/
   RCC->CFGR &=~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); //Reset
   //RCC->CFGR |= RCC_CFGR_PLLMULL9;      //PLL input clock x 9
   //RCC->CFGR |= RCC_CFGR_PLLXTPRE;      //HSE divider for PLL entry, 1: HSE clock divided by 2
   //RCC->CFGR |= RCC_CFGR_PLLSRC_1;    //PLL entry clock source, 1: HSE oscillator clock selected as PLL input clock
   //RCC->CR |= RCC_CR_PLLON;                     // Запустить PLL.
   //while (!(RCC->CR & RCC_CR_PLLRDY)) {} // Ожидание готовности PLL.
   RCC->CFGR &=(uint32_t)((uint32_t)~(RCC_CFGR_SW));
   RCC->CFGR |= RCC_CFGR_SW_PLL;                // Тактирование с выхода PLL.
   while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08) {} // Ожидание переключения на PLL.
   //RCC->APB1ENR |= RCC_APB1ENR_PWREN;
   RCC->CFGR |= RCC_CFGR_MCO_PLL;
   //RCC->CSR |= RCC_CSR_LSION;
   //while (!(RCC->CSR&RCC_CSR_LSIRDY)){}
   
   //DBGMCU->CR |= DBGMCU_CR_DBG_STANDBY;
}
static void RTC_Config(void)
 {
    RTC_InitTypeDef RTC_InitStructure;
    RTC_TimeTypeDef  RTC_TimeStruct;

    /* Allow access to RTC */
    PWR_BackupAccessCmd(ENABLE);
    
    /* Reset RTC Domain */
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);

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
    
    /* Set the time to 00h 00mn 00s AM */
    RTC_TimeStruct.RTC_H12     = RTC_H12_AM;
    RTC_TimeStruct.RTC_Hours   = 0x00;
    RTC_TimeStruct.RTC_Minutes = 0x00;
    RTC_TimeStruct.RTC_Seconds = 0x00;
    
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
    
    /* Enable the RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
    /* Wait for RTC APB registers synchronisation */
    RTC_WaitForSynchro();
    
 }

static void RTC_DateConfig(void)
{
  RTC_DateTypeDef RTC_DateStructure;
  RTC_DateStructInit(&RTC_DateStructure);
  RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
}
static void RTC_AlarmConfig(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  RTC_AlarmTypeDef RTC_AlarmStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RTC_AlarmStructInit(&RTC_AlarmStructure);
  
  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12      = RTC_H12_AM;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds  = 0x0A;
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel    = RTC_AlarmDateWeekDaySel_Date;
  RTC_AlarmStructure.RTC_AlarmDateWeekDay       = RTC_Weekday_Monday;    
  RTC_AlarmStructure.RTC_AlarmMask              = RTC_AlarmMask_DateWeekDay;
  
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
  
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
  
  RTC_OutputConfig(RTC_Output_AlarmA, RTC_OutputPolarity_High);
  RTC_OutputTypeConfig(RTC_OutputType_PushPull);

  /* Enable AlarmA interrupt */
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
  RTC_ClearITPendingBit(RTC_IT_ALRA);
} 
void RTC_IRQHandler(void)
{
  /* Check on the AlarmA flag and on the number of interrupts per Second (60*8) */
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET) 
  { 
    /* ALARM is enabled */
    ALARM_Occured = 1;
    /* Clear RTC AlarmA Flags */
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);
  }  
}
static void USART1_Config(void)
 {
   USART_InitTypeDef USART_InitStructure;
  
  /* USARTx configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - Stop Bit = 1 Stop Bit
  - Parity = No Parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
  
 }
   
//------------------------------------------------------------------------------
 
void Delay( unsigned int Val) {
  for( ; Val != 0; Val--) {
    __NOP();
  }
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
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
