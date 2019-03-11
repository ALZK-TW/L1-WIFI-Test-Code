//*****************************************************************************
// pinmux.c
//
// configure the device pins for different peripheral signals
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

// This file was automatically generated on 7/21/2014 at 3:06:20 PM
// by TI PinMux version 3.0.334
//
//*****************************************************************************

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

#define PAD_MODE_MASK        0x0000000F
#define PAD_STRENGTH_MASK    0x000000E0
#define PAD_TYPE_MASK        0x00000310

#define REG_PAD_CONFIG_26   (0x4402E108)
#define REG_PAD_CONFIG_27   (0x4402E10C)

//*****************************************************************************


void
PinMuxConfig(void)
{
   //
    // Enable Peripheral Clocks 
    //
  //  PRCMPeripheralClkEnable(PRCM_ADC, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);

    //==================== UART0  port setting ============================
    //
    // Configure PIN_55 for UART0 UART0_TX
    //
    PinTypeUART(PIN_55, PIN_MODE_3);

    //
    // Configure PIN_57 for UART0 UART0_RX
    //
    PinTypeUART(PIN_57, PIN_MODE_3);
 
    
    //==================== LED GPIO port setting ============================
    //
    // Configure PIN_50 for GPIO Output
    //
    PinTypeGPIO(PIN_50, PIN_MODE_0 , false); //LED RED (GPIO_00)
    
    GPIODirModeSet(GPIOA0_BASE, 0x1, GPIO_DIR_MODE_OUT);
    PinConfigSet(PIN_50,PIN_STRENGTH_14MA,PIN_TYPE_STD);
     //
    // Configure PIN_03 for GPIO Input
    //
    PinTypeGPIO(PIN_03, PIN_MODE_0, false);  // GPIO_12
    GPIODirModeSet(GPIOA1_BASE, 0x10,GPIO_DIR_MODE_OUT );// LED Yellow
    PinConfigSet(PIN_03,PIN_STRENGTH_14MA,PIN_TYPE_STD);
    
    //
    // Configure PIN_04 for GPIO Output
    //
    PinTypeGPIO(PIN_04, PIN_MODE_0 , false);  // GPIO_13
    GPIODirModeSet(GPIOA1_BASE, 0x20,GPIO_DIR_MODE_OUT );// LED Green
    PinConfigSet(PIN_04,PIN_STRENGTH_14MA,PIN_TYPE_STD);
     
   //==================== CC2650 UART1 RXD/TXD port setting ============================
   //
    // Configure PIN_01 for UART1 UART1_TX
    //
   
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);   //TXD1 CC2650 (GPIO_10)
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_IN);
    //
      // Configure PIN_02 for UART1 UART1_RX
    //
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);   //RXD1 CC2650 (GPIO_11)
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_IN);
    
    
    
    //==================== MOTOR IN/OUT GPIO port setting ============================
    //
    //
    // Configure PIN_53 for GPIO Input
    //
    PinTypeGPIO(PIN_53, PIN_MODE_0, false); // GPIO_30
    GPIODirModeSet(GPIOA3_BASE, 0x40, GPIO_DIR_MODE_OUT);//MOTOR OUT2
    // Configure PIN_58 for GPIO Output
    //
    PinTypeGPIO(PIN_62, PIN_MODE_0, false); //GPIO_03   //  Door position
    GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_IN);  // MOTOR SW sensor IN
    //
    // Configure PIN_63 for GPIO OUTPUT
    //
    PinTypeGPIO(PIN_63, PIN_MODE_0, false);//GPIO_08
    GPIODirModeSet(GPIOA1_BASE, 0x1, GPIO_DIR_MODE_OUT); //MOTOR OUT1
    
    //==================== Temperature sensor GPIO port setting ============================
    //
     //
    // Configure PIN_58 for GPIO Output
    //
   // PinTypeGPIO(PIN_58, PIN_MODE_0, false); //GPIO_03
   // GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_IN);  //Temp_Detect ADC in

    
   //==================== Cover switch port setting ============================
    //
    // Configure PIN_59 for GPIO Input
    //
    PinTypeGPIO(PIN_59, PIN_MODE_0, false);   //GPIO_04  // Tamper SW
    GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_IN);  // Cover SW (Hib wakeup)
    
    
    //==================== Battery ADC  port setting ============================
    //
    // Configure PIN_18 for GPIO Output
    //
    PinTypeGPIO(PIN_18, PIN_MODE_0, false);// GPIO_28(Battery voltage adc detect on/off control)
    GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);// Digital Out 
    //
    // Configure PIN_60 for ADC0 ADC_CH3
    //
    PinTypeADC(PIN_60, PIN_MODE_255); // Battery voltage Level ADC in
    
     //==================== Temperature  ADC  port setting ============================
    //
    // Configure PIN_17 for GPIO Output
    //
    PinTypeGPIO(PIN_17, PIN_MODE_0, false);// GPIO_28(Battery voltage adc detect on/off control)
    GPIODirModeSet(GPIOA3_BASE, 0x01, GPIO_DIR_MODE_OUT);// Digital Out 
    //
    // Configure PIN_58 for ADC0 ADC_CH1
    //
    PinTypeADC(PIN_58, PIN_MODE_255); // Battery voltage Level ADC in       
    
    //==================== Proximity Touch key  port setting ============================
    //
    //
    // Configure PIN_08 for GPIO In
    //
    PinTypeGPIO(PIN_08, PIN_MODE_0, false); // GPIO_17
    GPIODirModeSet(GPIOA2_BASE, 0x2, GPIO_DIR_MODE_IN); // Proximity Interrupt (Hib wakeup)
    //
    // Configure PIN_16 for GPIO Output Proximity power On/Off control
    //
    PinTypeGPIO(PIN_16, PIN_MODE_0, false);  // GPIO_23
    GPIODirModeSet(GPIOA2_BASE, 0x80,GPIO_DIR_MODE_OUT );    //Proximity power On/Off control  
        
    
    //==================== MFRC522 Mifare card reader  port setting ============================
    //
    //
    // Configure PIN_15 for GPIO Output
    //
    PinTypeGPIO(PIN_15, PIN_MODE_0, false);  // GPIO_22
    GPIODirModeSet(GPIOA2_BASE, 0x40,GPIO_DIR_MODE_OUT );    //MFRC522 H/W RESET 
    //-------------- I2C  port setting ----------------------------------------
    //
    // Configure PIN_05 for I2C0 I2C_SCL
    //
    PinTypeI2C(PIN_05, PIN_MODE_5);
    //
    // Configure PIN_06 for I2C0 I2C_SDA
    //
    PinTypeI2C(PIN_06, PIN_MODE_5);

    //-------------- Buzzer PWM port setting ----------------------------------------
    //    
    // Configure PIN_64 for TimerPWM5 GT_PWM05
    //
    PinTypeTimer(PIN_64, PIN_MODE_3);    // Buzzer
   //==================== Power source detect input ========================
    //
    //
   // Configure PIN_61 for GPIO Input
    //
    PinTypeGPIO(PIN_61, PIN_MODE_0, false);  //GPIO_06
    GPIODirModeSet(GPIOA0_BASE, 0x40, GPIO_DIR_MODE_IN);  //power source 



}

//*****************************************************************************
//
//! Configure Antenna Selection GPIOs
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void SetAntennaSelectionGPIOs(void)
{

   // MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
    MAP_GPIODirModeSet(GPIOA3_BASE,0xC,GPIO_DIR_MODE_OUT);
    
    //
    // Configure PIN_29 for GPIOOutput
    //    
    HWREG(REG_PAD_CONFIG_26) = ((HWREG(REG_PAD_CONFIG_26) & ~(PAD_STRENGTH_MASK 
                        | PAD_TYPE_MASK)) | (0x00000020 | 0x00000000 ));
    
    //
    // Set the mode.
    //
    HWREG(REG_PAD_CONFIG_26) = (((HWREG(REG_PAD_CONFIG_26) & ~PAD_MODE_MASK) |  
                                                    0x00000000) & ~(3<<10));
    
    //
    // Set the direction
    //
    HWREG(REG_PAD_CONFIG_26) = ((HWREG(REG_PAD_CONFIG_26) & ~0xC00) | 0x00000800);
    
    
     //
    // Configure PIN_30 for GPIOOutput
    //
    HWREG(REG_PAD_CONFIG_27) = ((HWREG(REG_PAD_CONFIG_27) & ~(PAD_STRENGTH_MASK
                                | PAD_TYPE_MASK)) | (0x00000020 | 0x00000000 ));
    
    //
    // Set the mode.
    //
    HWREG(REG_PAD_CONFIG_27) = (((HWREG(REG_PAD_CONFIG_27) & ~PAD_MODE_MASK) |  
                                        0x00000000) & ~(3<<10));

    //
    // Set the direction
    //
    HWREG(REG_PAD_CONFIG_26) = ((HWREG(REG_PAD_CONFIG_27) & ~0xC00) | 0x00000800);

    
}

//*****************************************************************************
//
//! GPIO Pin Write to Select Antenna 
//!
//! \param[in]  ucAntNum  1 - Antenna 1,2 - Antenna 2
//!
//! \return None
//
//*****************************************************************************
void AntennaSelect(unsigned char ucAntNum)
{
    if(ucAntNum == 1)
    {
         MAP_GPIOPinWrite(GPIOA3_BASE, 0xC, 0x8); // GPIO27
    }
    else if(ucAntNum == 2)
    {
        MAP_GPIOPinWrite(GPIOA3_BASE, 0xC, 0x4);  // GPIO26
    }

    return;
}