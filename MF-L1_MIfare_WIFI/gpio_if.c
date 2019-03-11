//*****************************************************************************
// gpio_if.c
//
// GPIO interface APIs, this common interface file helps to configure,
// set/toggle only 3 GPIO pins which are connected to 3 LEDs of CC32xx Launchpad
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_apps_rcm.h"
#include "interrupt.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
//#include "cc_pm.h"
//#include "cc_pm_ops.h"
#include "uart_if.h"
// OS includes
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
#include <stdlib.h>
#include "osi.h"
#endif

// Common interface include
#include "gpio_if.h"
#define Moto1_ON  ( GPIO_IF_Set(GPIO_MOTOR1, g_MOTOR1Port, g_MOTOR1Pin, 1))
#define Moto1_OFF  ( GPIO_IF_Set(GPIO_MOTOR1, g_MOTOR1Port, g_MOTOR1Pin, 0))

#define Moto2_ON (GPIO_IF_Set(GPIO_MOTOR2, g_MOTOR2Port, g_MOTOR2Pin, 1))
#define Moto2_OFF (GPIO_IF_Set(GPIO_MOTOR2, g_MOTOR2Port, g_MOTOR2Pin, 0))

//****************************************************************************
//                      GLOBAL VARIABLES                                   
//****************************************************************************
static unsigned long ulReg[]=
{
    GPIOA0_BASE,
    GPIOA1_BASE,
    GPIOA2_BASE,
    GPIOA3_BASE,
    GPIOA4_BASE
};

//*****************************************************************************
// Variables to store TIMER Port,Pin values
//*****************************************************************************
unsigned int g_uiLED1Port = 0,g_uiLED2Port = 0,g_uiLED3Port = 0;
unsigned char g_ucLED1Pin,g_ucLED2Pin,g_ucLED3Pin;

unsigned int g_uiI2CSCKPort = 0,g_uiI2CSDAPort = 0;
unsigned char g_ucI2CSCKPin,g_ucI2CSDAPin;

unsigned int g_MOTOR1Port = 0,g_MOTOR2Port = 0,g_MOTORSensPort=0;
unsigned char g_MOTOR1Pin,g_MOTOR2Pin,g_MOTORSensPin;

unsigned int g_TouchPWRENPort = 0,g_TouchPWRonoffPort = 0,g_TouchKeyINTPort;
unsigned char g_TouchPWRENPin,g_TouchPWRonoffPin,g_TouchKeyINTPin;

unsigned int g_KeyCoverSPort;
unsigned char g_KeyCoverSPin;

unsigned int g_BattDectPort = 0;
unsigned char g_BattDectPin;

unsigned int g_TempControlPort = 0;
unsigned char g_TempControlPin;


unsigned int g_RC522RSTPort = 0;
unsigned char g_RC522RSTPin;


unsigned int g_TouchKeyINTPort = 0;
unsigned char g_TouchKeyINTPin;

unsigned int g_PowerSourcePort = 0;
unsigned char g_PowerSourcePin;


//----------LED -------------------------
#define GPIO_LED1 0     // Red (Green)
#define GPIO_LED2 12    // Yellow
#define GPIO_LED3 13   // green(RED)



//--------------MOTOR --------------------------
#define GPIO_MOTOR1 8
#define GPIO_MOTOR2 30 
//#define GPIO_MOTOR_Sensor1 6
#define GPIO_MOTOR_Sensor 7

//-------------SET key -----------------------------
#define GPIO_KeyCoverS 4

//------- Battery voltage detect control port--------------------
#define GPIO_BattDect 28
//------- Temperature control port --------------------
#define GPIO_TempControl 24


//--------- Proximity touch wake up  ---------------
//#define GPIO_TouchPWREN 23
#define GPIO_TouchPWRonoff 23           
#define GPIO_TouchKeyINT 17             

//---------MFRC522 Mifare --------------------------
#define GPIO_RC522_RST 22               
//--------I2C ----------------------------
#define GPIO_I2CSCK 14
#define GPIO_I2CSDA 15 

//--------Power Source ----------------------------
#define GPIO_PowerSource 6


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************
unsigned char GPIO_IF_MotorSensStatus(void);
unsigned char GPIO_IF_TouchStatus(void);
void moto_unlock(void);
void moto_stop(void);
void moto_lock(void);

static unsigned char
GetPeripheralIntNum(unsigned int uiGPIOPort)
{

    switch(uiGPIOPort)
    {
       case GPIOA0_BASE:
          return INT_GPIOA0;
       case GPIOA1_BASE:
          return INT_GPIOA1;
       case GPIOA2_BASE:
          return INT_GPIOA2;
       case GPIOA3_BASE:
          return INT_GPIOA3;
       default:
          return INT_GPIOA0;
    }

}
//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  ucPins is the bit-pack representation of All in/out IO
//!         
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_Configure(void)
{

 /* Configure LED port */
    GPIO_IF_GetPortNPin(GPIO_LED1,
                        &g_uiLED1Port,
                        &g_ucLED1Pin);
 
    GPIO_IF_GetPortNPin(GPIO_LED2,
                  &g_uiLED2Port,
                  &g_ucLED2Pin);
  
    GPIO_IF_GetPortNPin(GPIO_LED3,
                      &g_uiLED3Port,
                      &g_ucLED3Pin);
    
    
 /* Configure I2C  port */
    /*
 
    GPIO_IF_GetPortNPin(GPIO_I2CSCK,
                       &g_uiI2CSCKPort,
                        &g_ucI2CSCKPin);
   
    GPIO_IF_GetPortNPin(GPIO_I2CSDA,
                  &g_uiI2CSDAPort,
          &g_ucI2CSDAPin);
    */
    
   /* Configure MOTOR control  port */

    GPIO_IF_GetPortNPin(GPIO_MOTOR1,
                        &g_MOTOR1Port,
                        &g_MOTOR1Pin);
    
    GPIO_IF_GetPortNPin(GPIO_MOTOR2,
                  &g_MOTOR2Port,
          &g_MOTOR2Pin);
    
    GPIO_IF_GetPortNPin(GPIO_MOTOR_Sensor,
                  &g_MOTORSensPort,
          &g_MOTORSensPin);
   
    
    GPIO_IF_GetPortNPin(GPIO_PowerSource,
              &g_PowerSourcePort,
      &g_PowerSourcePin);

    
 
    
   /* Configure Proximity touch port */

  //  GPIO_IF_GetPortNPin(GPIO_TouchPWREN,
  //                      &g_TouchPWRENPort,
   //                     &g_TouchPWRENPin);
    
    GPIO_IF_GetPortNPin(GPIO_TouchPWRonoff,
                  &g_TouchPWRonoffPort,
          &g_TouchPWRonoffPin);
    
    GPIO_IF_GetPortNPin(GPIO_TouchKeyINT,
                  &g_TouchKeyINTPort,
          &g_TouchKeyINTPin);
  
    
    
    /* Configure Set key port */
    GPIO_IF_GetPortNPin(GPIO_KeyCoverS,
                        &g_KeyCoverSPort,
                        &g_KeyCoverSPin);
   

     /* Battery ADC in on/off control port */
    GPIO_IF_GetPortNPin(GPIO_BattDect,
                        &g_BattDectPort,
                        &g_BattDectPin);
    
         /* Temperature ADC in on/off control port */
    GPIO_IF_GetPortNPin(GPIO_TempControl,
                        &g_TempControlPort,
                        &g_TempControlPin);

   
   /* Configure MFRC522 RESET  port */ 
   GPIO_IF_GetPortNPin(GPIO_RC522_RST,
                        &g_RC522RSTPort,
                        &g_RC522RSTPin);
   
      /* Configure Digital In/Out port */

   // GPIO_IF_GetPortNPin(GPIO_Dout,
    //                    &g_DoutPort,
     //                   &g_DoutPin);
    /*
    GPIO_IF_GetPortNPin(GPIO_Din1,
                  &g_Din1Port,
          &g_Din1Pin);
    
    GPIO_IF_GetPortNPin(GPIO_Din2,
                  &g_Din2Port,
          &g_Din2Pin);
    
  */
}

//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  ucPins is the bit-pack representation of 3 LEDs
//!         LSB:GP09-GP10-GP11:MSB
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_LedConfigure(unsigned char ucPins)
{

  if(ucPins & LED1)
  {
    GPIO_IF_GetPortNPin(GPIO_LED1,
                        &g_uiLED1Port,
                        &g_ucLED1Pin);
  }

  if(ucPins & LED2)
  {
    GPIO_IF_GetPortNPin(GPIO_LED2,
                  &g_uiLED2Port,
                  &g_ucLED2Pin);
  }

  if(ucPins & LED3)
  {
    GPIO_IF_GetPortNPin(GPIO_LED3,
                      &g_uiLED3Port,
                      &g_ucLED3Pin);

  }

}
//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  ucPins is the bit-pack representation of I2C
//!         LSB:GP09-GP10-GP11:MSB
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_I2CConfigure(unsigned char ucPins)
{

  if(ucPins & I2CSCK)
  {
    GPIO_IF_GetPortNPin(GPIO_I2CSCK,
                        &g_uiI2CSCKPort,
                        &g_ucI2CSCKPin);
  }

  if(ucPins & I2CSDA)
  {
    GPIO_IF_GetPortNPin(GPIO_I2CSDA,
                  &g_uiI2CSDAPort,
          &g_ucI2CSDAPin);
  }

  

}

//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  ucPins is the bit-pack representation of I2C
//!         LSB:GP09-GP10-GP11:MSB
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_MOTORportConfigure(unsigned char ucPins)
{

  if(ucPins & Motor1_PIN)
  {
    GPIO_IF_GetPortNPin(GPIO_MOTOR1,
                        &g_MOTOR1Port,
                        &g_MOTOR1Pin);
  }

  if(ucPins & Motor2_PIN)
  {
    GPIO_IF_GetPortNPin(GPIO_MOTOR2,
                  &g_MOTOR2Port,
          &g_MOTOR2Pin);
  }

  

}
void TouchKeyIntIsr()

{

Report("inside isrCheck1\n\r");

}

//*****************************************************************************
//
//! GPIO Enable & Configuration
//!
//! \param  
//!
//! \return None
//
//*****************************************************************************
void
GPIO_IF_TouchKeyIntConfigure(void)
{

  
    GPIO_IF_GetPortNPin(GPIO_TouchKeyINT,
                        &g_TouchKeyINTPort,
                        &g_TouchKeyINTPin);
    
    
  GPIO_IF_ConfigureNIntEnable(g_TouchKeyINTPort,
                                  g_TouchKeyINTPin,
                                  GPIO_FALLING_EDGE,
                                  TouchKeyIntIsr);
  

}

//****************************************************************************
//
//! \brief  This function handles the PRCM interrupt
//!
//! \param  intr_param is a void pointer (not used)
//!
//! \return none
//
//****************************************************************************
void wake_interrupt_handler() {
/*
    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t       CC3200 GPIO INTERRUPT handler       \n\r");
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");		
		
*/	
       GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);

  
  
}

//****************************************************************************
//
//! \brief  This function initialise the basic and application specific modules
//!         for a CC3200 platform
//!
//! \param  none
//!
//! \return 0 if application specific modules are loaded succesfully, -1 in case
//!         failure
//
//****************************************************************************
void GPIO_INT_init(void) {


	/* Register the interrupt with the global interrupt handler */
	MAP_IntRegister(INT_PRCM, wake_interrupt_handler);

	/* Enable the interrupt */
	MAP_IntEnable(INT_PRCM);

	MAP_PRCMIntEnable(PRCM_INT_SLOW_CLK_CTR);

	MAP_PRCMPeripheralClkEnable(PRCM_UDMA, PRCM_RUN_MODE_CLK |
	PRCM_SLP_MODE_CLK);

	
}


//*****************************************************************************
//
//! Turn LED On
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void
GPIO_IF_LedOff(char ledNum)
{
    switch(ledNum)
    {
        case MCU_ON_IND:
        case MCU_EXECUTE_SUCCESS_IND:
        case MCU_GREEN_LED_GPIO:
        {
          /* Switch ON GREEN LED */
          GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 0);
          break;
        }
        case MCU_SENDING_DATA_IND:
        case MCU_EXECUTE_FAIL_IND:
        case MCU_ORANGE_LED_GPIO:
        {
          /* Switch ON ORANGE LED */
          GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 0);
          break;
        }
        case MCU_ASSOCIATED_IND:
        case MCU_IP_ALLOC_IND:
        case MCU_SERVER_INIT_IND:
        case MCU_CLIENT_CONNECTED_IND:
        case MCU_RED_LED_GPIO:
        {
          /* Switch ON RED LED */
          GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 0);
          break;
        }
        case MCU_ALL_LED_IND:
        {
          /* Switch ON ALL LEDs LED */
          GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 0);
          GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 0);
          GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 0);
          break;
        }
        default:
          break;
    }
}

//*****************************************************************************
//
//! Turn LED Off
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void
GPIO_IF_LedOn(char ledNum)
{
  switch(ledNum)
  {
    case MCU_ON_IND:
    case MCU_EXECUTE_SUCCESS_IND:
    case MCU_GREEN_LED_GPIO:
    {
      /* Switch OFF GREEN LED */
      GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 1);
      break;
    }
    case MCU_SENDING_DATA_IND:
    case MCU_EXECUTE_FAIL_IND:
    case MCU_ORANGE_LED_GPIO:
    {
      /* Switch OFF ORANGE LED */
      GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 1);
      break;
    }
    case MCU_ASSOCIATED_IND:
    case MCU_IP_ALLOC_IND:
    case MCU_SERVER_INIT_IND:
    case MCU_CLIENT_CONNECTED_IND:
    case MCU_RED_LED_GPIO:
    {
      /* Switch OFF RED LED */
      GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 1);
      break;
    }
    case MCU_ALL_LED_IND:
    {
      /* Switch OFF ALL LEDs LED */
      GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 1);
      GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 1);
      GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 1);
      break;
    }
    default:
      break;
  }
}
//*****************************************************************************
//
//! Set Mifare Reset pin Hi or Low 
//!
//! \param  Hi_Low   1: Hi , 0 :Low
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void
MifareReset(char Hi_low)
{
   if(Hi_low)
     GPIO_IF_Set(GPIO_RC522_RST, g_RC522RSTPort, g_RC522RSTPin, 1);
   else 
     GPIO_IF_Set(GPIO_RC522_RST, g_RC522RSTPort, g_RC522RSTPin, 0);
}

//*****************************************************************************
//
//! set Battery ADC detect control port pin
//
//*****************************************************************************
void
BattVdectPin(char Hi_low)
{
   if(Hi_low)
     GPIO_IF_Set(GPIO_BattDect, g_BattDectPort, g_BattDectPin, 1);
   else 
     GPIO_IF_Set(GPIO_BattDect, g_BattDectPort, g_BattDectPin, 0);
}

//*****************************************************************************
//
//! set Temperarure ADC detect control port pin
//
//*****************************************************************************
void
TempControlPin(char Hi_low)
{
   if(Hi_low)
     GPIO_IF_Set(GPIO_TempControl, g_TempControlPort, g_TempControlPin, 1);
   else 
     GPIO_IF_Set(GPIO_TempControl, g_TempControlPort, g_TempControlPin, 0);
}
//*****************************************************************************
//
//! Set Mifare Reset pin Hi or Low 
//!
//! \param  Hi_Low   1: Hi , 0 :Low
//!
//! \return none
//!
//! \brief  Turns a specific LED Off
//
//*****************************************************************************
void
TVccOnOff(char Hi_low)
{
   if(Hi_low)
     GPIO_IF_Set(GPIO_TouchPWRonoff, g_TouchPWRonoffPort, g_TouchPWRonoffPin, 1);
   else 
     GPIO_IF_Set(GPIO_TouchPWRonoff, g_TouchPWRonoffPort, g_TouchPWRonoffPin, 0);
}


//*****************************************************************************
//
//! Turn I2C Off
//!
//!
//! \return none
//!
//
//*****************************************************************************
void
GPIO_IF_I2COff(char ledNum)
{
  
      /* Switch OFF I2C */
      GPIO_IF_Set(GPIO_I2CSCK, g_uiI2CSCKPort, g_ucI2CSCKPin, 1);
      GPIO_IF_Set(GPIO_I2CSDA, g_uiI2CSDAPort, g_ucI2CSDAPin, 1);
     
}
//*****************************************************************************
//
//! Turn I2C Off
//!
//!
//! \return none
//!
//
//*****************************************************************************
void
GPIO_IF_MOTOROff(void)
{
  
      /* Switch OFF MOTOR */
  Moto1_OFF;
  Moto2_OFF;
     
}
//*****************************************************************************
//
//! MOTOR control
//!
//!  1. Moto-Lock
//!  2. Moto_Unlock
//!  3. Moto_Stop
//!
//*****************************************************************************


void moto_lock(void)
{
  Moto1_OFF;
  Moto2_OFF;
  Moto1_ON;

   
}

void moto_unlock(void)
{
  Moto1_OFF;
  Moto2_OFF;
  Moto2_ON;
}

void moto_stop(void)
{
  Moto1_OFF;
  Moto2_OFF;
 
}
//*****************************************************************************
//
//!  \brief This function returns LED current Status
//!
//!  \param[in] ucGPIONum is the GPIO to which the LED is connected
//!                MCU_GREEN_LED_GPIO\MCU_ORANGE_LED_GPIO\MCU_RED_LED_GPIO
//!
//!
//!  \return 1: LED ON, 0: LED OFF
//
//*****************************************************************************
unsigned char
GPIO_IF_LedStatus(unsigned char ucGPIONum)
{
  unsigned char ucLEDStatus;
  switch(ucGPIONum)
  {
    case MCU_GREEN_LED_GPIO:
    {
      ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED3Port, g_ucLED3Pin);
      break;
    }
    case MCU_ORANGE_LED_GPIO:
    {
      ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED2Port, g_ucLED2Pin);
      break;
    }
    case MCU_RED_LED_GPIO:
    {
      ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED1Port, g_ucLED1Pin);
      break;
    }
    default:
        ucLEDStatus = 0;
  }
  return ucLEDStatus;
}
//*****************************************************************************
//
//!  \brief This function returns Motor SW current Status
//!
//!  \param[in]:Non
//!
//!
//!  \return 1: Moror SW open , 0: Motor SW Close
//
//*****************************************************************************
unsigned char
GPIO_IF_MotorSensStatus(void)
{
  unsigned char ucMotorSStatus;
 
      ucMotorSStatus = GPIO_IF_Get(GPIO_MOTOR_Sensor,g_MOTORSensPort,g_MOTORSensPin);

  return ucMotorSStatus;
}

//*****************************************************************************
//
//!  \brief This function returns Power source Status
//!
//!  \param[in]:Non
//!
//!
//!  \return 1: battery Power source  , 0: Power source AC/DC Adapter
//
//*****************************************************************************
unsigned char
GPIO_IF_PowerSource(void)
{
  unsigned char PowerSourceStatus;
 
      PowerSourceStatus = GPIO_IF_Get(GPIO_PowerSource,g_PowerSourcePort,g_PowerSourcePin);

  return PowerSourceStatus;
}


//*****************************************************************************
//
//! Toggle the Led state
//!
//! \param  ledNum is the LED Number
//!
//! \return none
//!
//! \brief  Toggles a board LED
//
//*****************************************************************************
void GPIO_IF_LedToggle(unsigned char ucLedNum)
{

    unsigned char ucLEDStatus = GPIO_IF_LedStatus(ucLedNum);
    if(ucLEDStatus == 1)
    {
        GPIO_IF_LedOff(ucLedNum);
    }
    else
    {
        GPIO_IF_LedOn(ucLedNum);
    }
}

//****************************************************************************
//
//! Get the port and pin of a given GPIO
//!
//! \param ucPin is the pin to be set-up as a GPIO (0:39)
//! \param puiGPIOPort is the pointer to store GPIO port address return value
//! \param pucGPIOPin is the pointer to store GPIO pin return value
//! 
//! This function  
//!    1. Return the GPIO port address and pin for a given external pin number
//!
//! \return None.
//
//****************************************************************************
void
GPIO_IF_GetPortNPin(unsigned char ucPin,
                unsigned int *puiGPIOPort,
                    unsigned char *pucGPIOPin)
{
    //
    // Get the GPIO pin from the external Pin number
    //
    *pucGPIOPin = 1 << (ucPin % 8);

    //
    // Get the GPIO port from the external Pin number
    //
    *puiGPIOPort = (ucPin / 8);
    *puiGPIOPort = ulReg[*puiGPIOPort];
}

//****************************************************************************
//
//! Configures the GPIO selected as input to generate interrupt on activity
//!
//! \param uiGPIOPort is the GPIO port address
//! \param ucGPIOPin is the GPIO pin of the specified port
//! \param uiIntType is the type of the interrupt (refer gpio.h)
//! \param pfnIntHandler is the interrupt handler to register
//! 
//! This function  
//!    1. Sets GPIO interrupt type
//!    2. Registers Interrupt handler
//!    3. Enables Interrupt
//!
//! \return None
//
//****************************************************************************
void
GPIO_IF_ConfigureNIntEnable(unsigned int uiGPIOPort,
                                  unsigned char ucGPIOPin,
                                  unsigned int uiIntType,
                                  void (*pfnIntHandler)(void))
{
    //
    // Set GPIO interrupt type
    //
    MAP_GPIOIntTypeSet(uiGPIOPort,ucGPIOPin,uiIntType);

    //
    // Register Interrupt handler
    //
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED) 
    // USE_TIRTOS: if app uses TI-RTOS (either networking/non-networking)
    // USE_FREERTOS: if app uses Free-RTOS (either networking/non-networking)
    // SL_PLATFORM_MULTI_THREADED: if app uses any OS + networking(simplelink)
    osi_InterruptRegister(GetPeripheralIntNum(uiGPIOPort),
                                        pfnIntHandler, INT_PRIORITY_LVL_1);
                
#else
	MAP_IntPrioritySet(GetPeripheralIntNum(uiGPIOPort), INT_PRIORITY_LVL_1);
    MAP_GPIOIntRegister(uiGPIOPort,pfnIntHandler);
#endif

    //
    // Enable Interrupt
    //
    MAP_GPIOIntClear(uiGPIOPort,ucGPIOPin);
    MAP_GPIOIntEnable(uiGPIOPort,ucGPIOPin);
}

//****************************************************************************
//
//! Set a value to the specified GPIO pin
//!
//! \param ucPin is the GPIO pin to be set (0:39)
//! \param uiGPIOPort is the GPIO port address
//! \param ucGPIOPin is the GPIO pin of the specified port
//! \param ucGPIOValue is the value to be set
//! 
//! This function  
//!    1. Sets a value to the specified GPIO pin
//!
//! \return None.
//
//****************************************************************************
void 
GPIO_IF_Set(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin,
             unsigned char ucGPIOValue)
{
    //
    // Set the corresponding bit in the bitmask
    //
    ucGPIOValue = ucGPIOValue << (ucPin % 8);

    //
    // Invoke the API to set the value
    //
    MAP_GPIOPinWrite(uiGPIOPort,ucGPIOPin,ucGPIOValue);
}

//****************************************************************************
//
//! Set a value to the specified GPIO pin
//!
//! \param ucPin is the GPIO pin to be set (0:39)
//! \param uiGPIOPort is the GPIO port address
//! \param ucGPIOPin is the GPIO pin of the specified port
//!
//! This function
//!    1. Gets a value of the specified GPIO pin
//!
//! \return value of the GPIO pin
//
//****************************************************************************
unsigned char
GPIO_IF_Get(unsigned char ucPin,
             unsigned int uiGPIOPort,
             unsigned char ucGPIOPin)
{
    unsigned char ucGPIOValue;
    long lGPIOStatus;

    //
    // Invoke the API to Get the value
    //
    lGPIOStatus =  MAP_GPIOPinRead(uiGPIOPort,ucGPIOPin);

    //
    // Set the corresponding bit in the bitmask
    //
    ucGPIOValue = lGPIOStatus >> (ucPin % 8);
    return ucGPIOValue;
}

//*****************************************************************************
//
//!  \brief This function returns KeyCover SW current Status
//!
//!  \param[in] No
//!
//!
//!  \return 1: Key Cover close 0: Key Cover open
//
//*****************************************************************************
unsigned char
GPIO_IF_KeyCoverSStatus(void)
{
  unsigned char ucKeyCoverSStatus;
 
      ucKeyCoverSStatus = GPIO_IF_Get(GPIO_KeyCoverS,g_KeyCoverSPort,g_KeyCoverSPin);

          
  return ucKeyCoverSStatus;
}
 
//*****************************************************************************
//
//!  \brief This function returns Proximity  current Status
//!
//!  \param[in] No
//!
//!
//!  \return 1: Proximity INT  release 0: Proximity INT occur
//
//*****************************************************************************
unsigned char
GPIO_IF_TouchStatus(void)
{
  unsigned char ucTouchStatus;
 
      ucTouchStatus = GPIO_IF_Get(GPIO_TouchKeyINT,g_TouchKeyINTPort,g_TouchKeyINTPin);

  return ucTouchStatus;
}
 

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
