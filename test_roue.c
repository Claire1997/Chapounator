/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
 * File Name          : main.c
 * Author             : danceww
 * Version            : V0.0.1
 * Date               : 08/23/2010
 * Description        : Main program body
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/

//#include "includes.h"
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"

#include "AX12.h"
#include "AXS1.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46

#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB


#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11


#define PORT_LED_AUX			GPIOB
#define PORT_LED_MANAGE			GPIOB
#define PORT_LED_PROGRAM		GPIOB
#define PORT_LED_PLAY			GPIOB
#define PORT_LED_POWER			GPIOC
#define PORT_LED_TX				GPIOC
#define PORT_LED_RX				GPIOC

#define PIN_LED_AUX				GPIO_Pin_12
#define PIN_LED_MANAGE			GPIO_Pin_13
#define PIN_LED_PROGRAM			GPIO_Pin_14
#define PIN_LED_PLAY			GPIO_Pin_15
#define PIN_LED_POWER			GPIO_Pin_13
#define PIN_LED_TX				GPIO_Pin_14
#define PIN_LED_RX				GPIO_Pin_15


#define USART_DXL			    0
#define USART_PC			    2

#define word                    u16
#define byte                    u8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile byte                   gbpRxInterruptBuffer[256]; // dxl buffer
volatile byte                   gbRxBufferWritePointer,gbRxBufferReadPointer;
volatile vu32                   gwTimingDelay,gw1msCounter;
u32                             Baudrate_DXL = 	1000000;
u32                             Baudrate_PC = 57600;
vu16                            CCR1_Val = 100; 		// 1ms
vu32                            capture = 0;
word                            GoalPos[2] = {0, 1023};
//word                            GoalPos[2] = {0, 1023};  //For EX and MX series
word                            Position;
word                            wPresentPos;
byte                            INDEX = 0;
byte                            Voltage;
byte                            id = 1;
byte                            bMoving, CommStatus;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void Timer_configuration(void);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void __ISR_DELAY(void);
void USART1_Configuration(u32);
void USART_Configuration(u8, u32);
void DisableUSART1(void);
void ClearBuffer256(void);
byte CheckNewArrive(void);
void PrintCommStatus(int);
void PrintErrorCode(void);
void TxDByte_DXL(byte);
byte RxDByte_DXL(void);
void TxDString(byte*);
void TxDWord16(word);
void TxDByte16(byte);
void TxDByte_PC(byte);
void Timer_Configuration(void);
void mDelay(u32);
void StartDiscount(s32);
byte CheckTimeOut(void);


////////////////////////////////////////////////////////////
///////////////////// value definitions ////////////////////
////////////////////////////////////////////////////////////

// change IDs here if the IDs of your motors/sensors are different
#define MOTOR 2
#define SENSOR 100


// state machine constants
#define INIT 0
#define GO_TO_CENTER 1
#define SEEKING 2
#define CHASING 3
#define STOP -1


#define speed_ini 400
#define speed_max 512


//////////////////////////////////////
/////////////// AX 12 ////////////////
//////////////////////////////////////

// infinite turn mode activation, see technical docu
// parameter: ID of motor
void infiniteTurn(unsigned char id) {
  dxl_write_word(id,  AX12_CTAB_ID_CWAngleLimitLo, 0 ) ;
  int result =  dxl_get_result();
  dxl_write_word(id,  AX12_CTAB_ID_CCWAngleLimitLo, 0 ) ;
  result =  dxl_get_result();
  TxDString("\nCM5 infinite rotation mode set\n");
}


// infinite turn mode desactivation, see technical docu
// parameter: ID of motor
void normalTurn(unsigned char id) {
  dxl_write_word(id,  AX12_CTAB_ID_CWAngleLimitLo, 0 ) ;
  int result =  dxl_get_result();
  dxl_write_word(id,  AX12_CTAB_ID_CCWAngleLimitLo, 1023 ) ;
  result =  dxl_get_result();
  TxDString("\nCM5 normal rotation mode set\n");
}


// set rotation speed of a single motor, only works in infinite turn mode!
// speed is an integer between -1023 and 1023
// parameter motor: ID of motor
// parameter speed: rotation speed, between -1024 and 1024, sign controls direction
// speed 1 = no ratation, speed 0 = maximal speed
void setSpeed(unsigned char id, int speed) {
  int order;
  if(speed >= 0)
    order = speed;
  else
    order = 1024 - speed;
  dxl_write_word(id, AX12_CTAB_ID_MovingSpeedLo, order ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("problem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}

// move motor to a given angle, only works when nOt in infinite turn mode
// parameter motor: ID of motor
// parameter: angle is an integer between -1023 and 1023
// no angle should be between 300 and 360 degrees
void setAngle(unsigned char id, int angle, int speed) {
  setSpeed(id, speed);
  int angle_norm;

  if (angle >=0)
    angle_norm = angle;

  else
    angle_norm = 1024 + angle;

  dxl_write_word(id,  AX12_CTAB_ID_GoalPositionLo, angle_norm ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code==");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}


// turns on motor light
// parameter motor: ID of motor
void lightOn(unsigned char id) {
  dxl_write_byte(id, AX12_CTAB_ID_Led, 1 ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code==");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}

// turns off motor light
// parameter motor: ID of motor
void lightOff(unsigned char id) {
  dxl_write_byte(id, AX12_CTAB_ID_Led, 0 ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code==");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}

// returns the current motor's speed
// This functions does not return anything but stores the speed in its 2nd parameter which lust be a pointer to int
// parameter inId: ID of motor
// parameter outSpeed: pointer to which the speed will be stored
void getSpeed(unsigned char id, unsigned int* outSpeed) {
  *outSpeed = dxl_read_word(id, AX12_CTAB_ID_MovingSpeedLo) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}


// returns the current motor's angle,  infinite turn must be disabled to use this function
// This functions does not return anything but stores the speed in its 2nd parameter which lust be a pointer to int
// parameter inId: ID of motor
// parameter outSpeed: pointer to which the speed will be stored
void getAngle(unsigned char id, unsigned int* outAngle) {
  *outAngle = dxl_read_word(id, AX12_CTAB_ID_PresentPosLo) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}

/////////////////////////////////////////////////////
////////////////////////////// AX S1 ////////////////
/////////////////////////////////////////////////////

// returns the obstacle detection flag (using infrared sensors), see technical documentation
// parameter sensor: Id of AX-S1
// parameter boolLight: pointer to store data read from AX-S1
void checkObstacle(unsigned char sensor, unsigned char* infoObst) {
  *infoObst = dxl_read_byte(sensor, AXS1_CTAB_ID_ObstacleDetectionFlag) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }

}


// returns the light detection flag (using visual light sensors), see technical documentation
// parameter sensor: Id of AX-S1
// parameter boolLight: pointer to store data read from AX-S1
void checkLuminosity(unsigned char sensor, unsigned char* info)  {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_LuminosityDetectionFlag) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
     }

}
// returns the left infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void leftInfraRed(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_LeftIRSensorData) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}

// returns the center infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void centerInfraRed(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_CenterIRSensorData) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
    }
}

// returns the right infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void rightInfraRed(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor, AXS1_CTAB_ID_RightIRSensorData) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
     }
}


// returns the left leight sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter leftLum: pointer to store data read from AX-S1
void leftLuminosity(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor,AXS1_CTAB_ID_LeftLuminosity ) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
     }
}

// returns the central light sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter centerLum: pointer to store data read from AX-S1
void centerLuminosity(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor,AXS1_CTAB_ID_CenterLuminosity ) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
     }
}

// returns the right light sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter rightLum: pointer to store data read from AX-S1
void rightLuminosity(unsigned char sensor, unsigned char* info) {
  *info = dxl_read_byte(sensor,AXS1_CTAB_ID_RightLuminosity ) ;
  int result =  dxl_get_result();
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}


// returns the amount of sound detected
// untested, see documentation of AX-S1!!
void dataSound(unsigned char sensor, unsigned int* info) {
  *info = dxl_read_word(sensor,AXS1_CTAB_ID_SoundData) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
     }
}


//helper function
void noteBuzz(unsigned char sensor, int note) {
  dxl_write_byte(sensor, AXS1_CTAB_ID_BuzzerIndex, note) ;
   int result =  dxl_get_result();
   if( result != COMM_RXSUCCESS	)
     {
       TxDString("\nproblem, code=");
       TxDWord16(result);
       TxDString("!!!\n");
     }
}

//helper function
void timeBuzz(unsigned char sensor, int time) {
  int result=-1 ;
  while (result != COMM_RXSUCCESS)
  {
    dxl_write_byte(sensor, AXS1_CTAB_ID_BuzzerTime, time) ;
    result =  dxl_get_result();
  }
  if( result != COMM_RXSUCCESS	)
    {
      TxDString("\nproblem, code=");
      TxDWord16(result);
      TxDString("!!!\n");
    }
}

// play a note of given duration on the AX-S1.
// tim is in milliseconds, so 500 means half a second
void buzzWithDelay(unsigned char sensor, int note, int time) {
  int k = 0;
  // infinite duration buzz
  timeBuzz(sensor,254);
  noteBuzz(sensor, note);

  for (k=0; k<time; k++) {
       mDelay(1) ;
  }

  // shut off buzz
  timeBuzz(sensor,0) ;
  return ;

}

/////////////////////////////////////////////////////////////////////////
//////////////////////////   M A I N   L O O P   ////////////////////////
/////////////////////////////////////////////////////////////////////////


int main(void)
{
  // --------------DO NOT TOUCH!!------------------------ //
  // NEVER!!! EVER!!!

  /* System Clocks Configuration */
  RCC_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();

  /* GPIO configuration */
  GPIO_Configuration();

  SysTick_Configuration();

  Timer_Configuration();

  dxl_initialize( 0, 1 );
  USART_Configuration(USART_PC, Baudrate_PC);
  // -----------------UNTIL HERE------------------------- //

  // here, touch all u like :-)
  //
  int thresholdInfrared = 50 ;
  // state will define the robot's attitude towards its environment
  // it implements a state machine
  // thats why we call it state :-)
  int state;


  state= INIT;
  infiniteTurn(MOTOR);
  unsigned char field;

  //  printf("Resetting motors\n") ;
  setSpeed(MOTOR, 0);


  // state should equal INIT only at the beginning of each match

  while(state!=STOP)
  { 
       state=GO_TO_CENTER;
       TxDString("\nDémarrage\n") ;

     while (state==GO_TO_CENTER) {  // the temporisation should be adapted

       setSpeed(MOTOR, 1023);
     }
  }

  while (1) {} ;

  return 0;
}

// --------------DO NOT TOUCH!!------------------------ //
// NEVER!!! EVER!!!

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
    {
      /* Enable Prefetch Buffer */
      FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

      /* Flash 2 wait state */
      FLASH_SetLatency(FLASH_Latency_2);

      /* HCLK = SYSCLK */
      RCC_HCLKConfig(RCC_SYSCLK_Div1);

      /* PCLK2 = HCLK */
      RCC_PCLK2Config(RCC_HCLK_Div1);

      /* PCLK1 = HCLK/2 */
      RCC_PCLK1Config(RCC_HCLK_Div2);

      /* PLLCLK = 8MHz * 9 = 72 MHz */
      RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

      /* Enable PLL */
      RCC_PLLCmd(ENABLE);

      /* Wait till PLL is ready */
      while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

      /* Wait till PLL is used as system clock source */
      while(RCC_GetSYSCLKSource() != 0x08)
	{
	}
    }

  /* Enable peripheral clocks --------------------------------------------------*/

  /* Enable USART1 and GPIOB clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);

  /* Enable USART3 clocks */
  RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2, ENABLE);

  PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configures Vector Table base location.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  // Set the Vector Table base location at 0x20000000
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
  // Set the Vector Table base location at 0x08003000
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
#endif

  // Configure the NVIC Preemption Priority Bits
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable the TIM2 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : GPIO_Configuration
 * Description    : Configures the different GPIO ports.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  // PORTB CONFIG
  GPIO_InitStructure.GPIO_Pin = 	PIN_ENABLE_TXD | PIN_ENABLE_RXD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
  GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);

  GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
  GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}

void USART1_Configuration(u32 baudrate)
{
  USART_Configuration(USART_DXL, baudrate);
}

void USART_Configuration(u8 PORT, u32 baudrate)
{

  USART_InitTypeDef USART_InitStructure;

  USART_StructInit(&USART_InitStructure);


  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  if( PORT == USART_DXL )
    {
      USART_DeInit(USART1);
      mDelay(10);
      /* Configure the USART1 */
      USART_Init(USART1, &USART_InitStructure);

      /* Enable USART1 Receive and Transmit interrupts */
      USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
      //USART_ITConfig(USART1, USART_IT_TC, ENABLE);

      /* Enable the USART1 */
      USART_Cmd(USART1, ENABLE);
    }

  else if( PORT == USART_PC )
    {
      USART_DeInit(USART3);
      mDelay(10);
      /* Configure the USART3 */
      USART_Init(USART3, &USART_InitStructure);

      /* Enable USART3 Receive and Transmit interrupts */
      //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
      //USART_ITConfig(USART3, USART_IT_TC, ENABLE);

      /* Enable the USART3 */
      USART_Cmd(USART3, ENABLE);
    }
}

void DisableUSART1(void)
{
  USART_Cmd(USART1, DISABLE);
}

void ClearBuffer256(void)
{
  gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}

byte CheckNewArrive(void)
{
  if(gbRxBufferReadPointer != gbRxBufferWritePointer)
    return 1;
  else
    return 0;
}

void TxDByte_DXL(byte bTxdData)
{
  GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
  GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

  USART_SendData(USART1,bTxdData);
  while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

  GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
  GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}

byte RxDByte_DXL(void)
{
  byte bTemp;

  while(1)
    {
      if(gbRxBufferReadPointer != gbRxBufferWritePointer) break;
    }

  bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
  gbRxBufferReadPointer++;

  return bTemp;
}


// Print communication result
void PrintCommStatus(int CommStatus)
{
  switch(CommStatus)
    {
    case COMM_TXFAIL:
      TxDString("COMM_TXFAIL: Failed transmit instruction packet!\n");
      break;

    case COMM_TXERROR:
      TxDString("COMM_TXERROR: Incorrect instruction packet!\n");
      break;

    case COMM_RXFAIL:
      TxDString("COMM_RXFAIL: Failed get status packet from device!\n");
      break;

    case COMM_RXWAITING:
      TxDString("COMM_RXWAITING: Now recieving status packet!\n");
      break;

    case COMM_RXTIMEOUT:
      TxDString("COMM_RXTIMEOUT: There is no status packet!\n");
      break;

    case COMM_RXCORRUPT:
      TxDString("COMM_RXCORRUPT: Incorrect status packet!\n");
      break;

    default:
      TxDString("This is unknown error code!\n");
      break;
    }
}

// Print error bit of status packet
void PrintErrorCode()
{
  if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    TxDString("Input voltage error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    TxDString("Angle limit error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    TxDString("Overheat error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    TxDString("Out of range error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    TxDString("Checksum error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    TxDString("Overload error!\n");

  if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    TxDString("Instruction code error!\n");
}

void TxDString(byte *bData)
{
  while (*bData)
    TxDByte_PC(*bData++);
}

void TxDWord16(word wSentData)
{
  TxDByte16((wSentData >> 8) & 0xff);
  TxDByte16(wSentData & 0xff);
}

void TxDByte16(byte bSentData)
{
  byte bTmp;

  bTmp = ((byte) (bSentData >> 4) & 0x0f) + (byte) '0';
  if (bTmp > '9')
    bTmp += 7;
  TxDByte_PC(bTmp);
  bTmp = (byte) (bSentData & 0x0f) + (byte) '0';
  if (bTmp > '9')
    bTmp += 7;
  TxDByte_PC(bTmp);
}

void TxDByte_PC(byte bTxdData)
{
  USART_SendData(USART3,bTxdData);
  while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
}

void Timer_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);

  TIM_DeInit(TIM2);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val ;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

void TimerInterrupt_1ms(void) //OLLO CONTROL
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
    {
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

      capture = TIM_GetCapture1(TIM2);
      TIM_SetCompare1(TIM2, capture + CCR1_Val);

      if(gw1msCounter > 0)
	gw1msCounter--;
    }
}

/*__interrupt*/
void RxD0Interrupt(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}

void SysTick_Configuration(void)
{
  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
  SysTick_SetReload(9000);

  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);
}

void __ISR_DELAY(void)
{
  if (gwTimingDelay != 0x00)
    gwTimingDelay--;
}

void mDelay(u32 nTime)
{
  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);

  gwTimingDelay = nTime;

  while(gwTimingDelay != 0);

  /* Disable SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Disable);
  /* Clear SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}

void StartDiscount(s32 StartTime)
{
  gw1msCounter = StartTime;
}

u8 CheckTimeOut(void)
{
  // Check timeout
  // Return: 0 is false, 1 is true(timeout occurred)

  if(gw1msCounter == 0)
    return 1;
  else
    return 0;
}
