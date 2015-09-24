#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "diskio.h"
#include "ff.h"
#include "rtc_32f1.h"
#include "xprintf.h"
#define F_CPU           72000000UL
#define TimerTick	F_CPU/1000-1 // 1 kHz

extern void disk_timerproc(void);

DWORD AccSize;				/* Work register for fs command */
WORD AccFiles, AccDirs;
FILINFO Finfo;
FRESULT result;
UINT nRead, nWritten;
DWORD ofs_crs = 0;
unsigned char sleep = 0;
unsigned char sh;
//char buff[255] = {0};


#if _USE_LFN
char Lfname[512];

#endif

char Line[256];				/* Console input buffer */
BYTE Buff[4096]; //__attribute__ ((aligned (4))) ;	/* Working buffer */

RTCTIME rtc;
FATFS FatFs;				/* File system object for each logical drive */
FIL file;				/* File objects */
DIR dir;					/* Directory object */
DWORD count_ms;
char cmd[256];
int num = 0;


volatile UINT Timer;

volatile uint32_t cnt_puls_pos = 0;
volatile uint32_t cnt_puls_neg = 0;
volatile unsigned char flag =0;
//------------------------------------------------------------------------------
void InitSysClock(void);
void InitGPIO(void);
void Delay(unsigned int Val);
void InitIRQ(void);
void InitUART(void);
void InitSysTick(void);
void UART1_Tx(unsigned char x);
void UART1_Tx_Str(char *buf);
void InitRTC(void);
void InitIWDG(void);
void put_rc(FRESULT rc);
int StrToInt(char *ptr);
 
//------------------------------------------------------------------------------
 
int main( void) 
{
  char *ptr2;
  //char *ptr;
  InitSysClock();
  InitGPIO();
  InitSysTick();
  InitIRQ();
  InitUART();
  //NVIC_EnableIRQ (EXTI1_IRQn);
  //NVIC_EnableIRQ (EXTI2_IRQn);
  NVIC_EnableIRQ (USART1_IRQn);
  //rtc_initialize();
  //rtc_gettime(&rtc);
  if (rtc_initialize()) 
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
  
  __enable_irq ();
  while(1) 
  {
    ptr2 = cmd; 
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
    }
  }
}
//-----------------------------------------------------------------------------
void InitIRQ(void)
{
  //External interrupt
  //Interrupt INT1 for PORTA.
  AFIO->EXTICR[1] |= AFIO_EXTICR1_EXTI1_PA;     
  EXTI->FTSR |= EXTI_FTSR_TR1;
  EXTI->IMR |= EXTI_IMR_MR1; 
  
  //External interrupt
  //Interrupt INT2 for PORTA.
  AFIO->EXTICR[1] |= AFIO_EXTICR1_EXTI2_PA;     
  EXTI->FTSR |= EXTI_FTSR_TR2;
  EXTI->IMR |= EXTI_IMR_MR2;
  
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
  if (count_ms == 1000)
  {
    count_ms = 0;
    if ((!cnt_puls_neg)&&(!cnt_puls_pos)&&(sleep))
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
            sprintf(buff, "%u/%u/%u %2u:%02u:%02u P %d\r\n", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec, cnt_puls_pos);
            UART1_Tx_Str(buff);
            int str_lenth = strlen(buff);
            f_lseek(&file, f_size(&file));
            //f_lseek(&file, ofs_crs);
            f_write(&file, buff, str_lenth, &nWritten);
            f_close(&file);
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
            sprintf(buff, "%u/%u/%u %2u:%02u:%02u N %d\r\n", rtc.mday, rtc.month, rtc.year, rtc.hour, rtc.min, rtc.sec, cnt_puls_pos);
            UART1_Tx_Str(buff);
            int str_lenth = strlen(buff);
            ofs_crs = f_tell(&file);
            f_lseek(&file, ofs_crs);
            f_write(&file, buff, str_lenth, &nWritten);
            //ofs_crs += str_lenth;
            f_close(&file);
          }
        else {put_rc(result);}
        cnt_puls_neg = 0;
      }
    }
  }
  disk_timerproc();
}

DWORD get_fattime (void)
{
        RTCTIME rtc;
	/* Get local time */
	if (!rtc_gettime(&rtc)) return 0;
	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.year - 1980) << 25)
		| ((DWORD)rtc.month << 21)
		| ((DWORD)rtc.mday << 16)
		| ((DWORD)rtc.hour << 11)
		| ((DWORD)rtc.min << 5)
		| ((DWORD)rtc.sec >> 1);
}

void EXTI1_IRQHandler(void)
{
  Delay(50);
  cnt_puls_pos += 1;
  EXTI->PR |= EXTI_PR_PR1;
}

void EXTI2_IRQHandler(void)
{
  Delay(5);
  cnt_puls_neg += 1;
  EXTI->PR |= EXTI_PR_PR2;
}

void USART1_IRQHandler(void)
{
 if (USART1->SR & USART_SR_RXNE)
	{
	cmd[num]= USART1->DR;
        if (cmd[num]=='\0'){
          num = 0;
        }
        else {num += 1;}
	}
}
void InitUART(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  USART1->BRR = 0x341; //0xEA60;		// Bodrate for 1200 on 72Mhz
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // USART1 ON, TX ON, RX ON
}

void InitGPIO( void) 
{
  // Enable PORTB Periph clock  
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  /* Config PortB 8*/
  /*GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8); // reset config PortB8
  GPIOB->CRH |= GPIO_CRH_MODE8_1;
  GPIOB->BSRR = GPIO_BSRR_BS8;*/
  /* Config PortB 0*/
  GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // reset config PortB0
  GPIOB->CRL |= GPIO_CRL_MODE0_1;
  GPIOB->BSRR = GPIO_BSRR_BS0;
  // Enable PORTC Periph clock  
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  // For GREEN LED PC10 Output
  GPIOC->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
  GPIOC->CRH |= GPIO_CRH_MODE10_1;
  // Enable PORTA Periph clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;;
  
  //For Ext.Interrupt PA1 Input, resistor Pull-Down
  GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
  GPIOA->CRL |= GPIO_CRL_CNF1_1;
  
  //For Ext.Interrupt PA2 Input, resistor Pull-Down
  GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
  GPIOA->CRL |= GPIO_CRL_CNF2_1;
  
  //For UART. PA9 Output, PA10 Input
  GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
  GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
  GPIOA->CRH |= GPIO_CRH_CNF9_1;         //PA9 alternate function output Push-pull
  GPIOA->CRH |= GPIO_CRH_MODE9_0;       //PA9 output mode, max speed 10 MHz
  GPIOA->CRH |= GPIO_CRH_CNF10_1 ;        //PA10 input with pull-up / pull-down 
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
void InitSysClock(void)
{
   RCC->CR |= RCC_CR_HSEON;                       // Включить генератор HSE.
   while (!(RCC->CR & RCC_CR_HSERDY)) {};       // Ожидание готовности HSE. 
   //RCC->CFGR2 &=~(RCC_CFGR2_PREDIV1);         // Предочистка делителя HSE.
   /*RCC->CFGR &=~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); // Предочистка.
   RCC->CFGR |= RCC_CFGR_PLLSRC_PREDIV1;        // Тактировать PLL от HSE/PREDIV1.
   RCC->CFGR |= RCC_CFGR_PLLMULL9;            //Умножать частоту на PLL_MUL.
   RCC->CR |= RCC_CR_PLLON;                     // Запустить PLL.
   while (!(RCC->CR & RCC_CR_PLLRDY)) {} // Ожидание готовности PLL.
   FLASH->ACR |= FLASH_ACR_PRFTBE;              // Enable Prefetch Buffer.
   FLASH->ACR |= FLASH_ACR_LATENCY_2;           // Если 48< SystemCoreClock <= 72, пропускать 2 такта.*/
   RCC->CR |= RCC_CR_PLLON;                     // Запустить PLL.
   while (!(RCC->CR & RCC_CR_PLLRDY)) {} // Ожидание готовности PLL.
   RCC->CFGR &=~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); //Reset
   RCC->CFGR |= RCC_CFGR_PLLMULL9;      //PLL input clock x 9
   RCC->CFGR |= RCC_CFGR_PLLXTPRE;      //HSE divider for PLL entry, 1: HSE clock divided by 2
   RCC->CFGR |= RCC_CFGR_PLLSRC_HSE;    //PLL entry clock source, 1: HSE oscillator clock selected as PLL input clock
   //RCC->CR |= RCC_CR_CSSOFF;
   RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
   RCC->CFGR |= RCC_CFGR_SW_PLL;                // Тактирование с выхода PLL.
   while ((RCC->CFGR & RCC_CFGR_SWS) != 0x08) {} // Ожидание переключения на PLL.
   RCC->APB1ENR |= RCC_APB1ENR_PWREN;
   //RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
   //RCC->CSR |= RCC_CSR_LSION;
   //while (!(RCC->CSR&RCC_CSR_LSIRDY)){}
   
   //DBGMCU->CR |= DBGMCU_CR_DBG_STANDBY;
}
  
 
//------------------------------------------------------------------------------
 
void Delay( unsigned int Val) {
  for( ; Val != 0; Val--) {
    __NOP();
  }
}
void UART1_Tx(unsigned char x)
{
  while (!(USART1->SR & USART_SR_TXE)); //Ожидаем освобождения буферного регистра TDR
  USART1->DR = x;                       // Отправляем символ "F".
  Delay(2000);
}
void UART1_Tx_Str(char *buf)
{
  while(*buf!='\0')
  {
    UART1_Tx(*buf++);
  } 
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
	UART1_Tx_Str(str);
}
