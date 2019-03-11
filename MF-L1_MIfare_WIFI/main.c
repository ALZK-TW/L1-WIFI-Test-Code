//*****************************************************************************
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
// Ver 0.02   Add Un_lock() & Lock() function Call  --- 0919/2017
// 
// Standard includes
#include <stdlib.h>
#include <string.h>
// test
// simplelink includes 
#include "simplelink.h"
#include "wlan.h"

//Free_rtos/ti-rtos includes
#include "osi.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"
//mwang
#include "adc.h"
#include "pin.h"

// common interface includes 
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "pinmux.h"
//mwang
#include "gpio.h"
//Cavin
#include "gpio_if.h"
#include "i2c_if.h"
#include "rc522.h"
#include "Utils_if.h"

// cavin  add timer for PWM
#include "timer.h"
#include "timer_if.h"

// Cavin
#define LED_KEY_R        0
#define LED_KEY_Y        1 
#define LED_KEY_G        2          

// to Adjust the buzzer volume by change the Duty value (big :128 mid:32 small:16 mute:0)
#define Buzzer_VB        128     // Sound big
#define Buzzer_VM        32     // Sound mid
#define Buzzer_VS        16     // Sound small
#define Buzzer_Voff       0     // Sound  off 

// Door_Direction
#define Right        1
#define Left         0   

//File System
#define SL_MAX_FILE_SIZE        1024L       
#define BUF_SIZE                16
#define USER_FILE_NAME          "/cert/client.pem"

#define OSI_STACK_SIZE      2048

/* Application specific status/error codes */
typedef enum{
    // Choosing this number to avoid overlap w/ host-driver's error codes
    FILE_ALREADY_EXIST = -0x7D0,
    FILE_CLOSE_ERROR = FILE_ALREADY_EXIST - 1,
    FILE_NOT_MATCHED = FILE_CLOSE_ERROR - 1,
    FILE_OPEN_READ_FAILED = FILE_NOT_MATCHED - 1,
    FILE_OPEN_WRITE_FAILED = FILE_OPEN_READ_FAILED -1,
    FILE_READ_FAILED = FILE_OPEN_WRITE_FAILED - 1,
    FILE_WRITE_FAILED = FILE_READ_FAILED - 1,
    SOCKET_CREATE_ERROR = FILE_WRITE_FAILED -1,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    SEND_ERROR = BIND_ERROR - 1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE = RECV_ERROR -1,  
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef enum{
    // The open file type for writing in flash
    CREATE_WRITE = 0,
    EXISTING_WRITE = 1
}e_WriteType;

typedef union BatteryTestCount
{
    unsigned char para[4];
    unsigned int data;        
}BatteryTestData; 

typedef struct InpCmdVectorStruct
{
  unsigned short usCmd;
  unsigned char usAID;
  unsigned char ucCardMasterKey[16]; 
  unsigned char ucMasterKey[16]; //  AMK key
  unsigned char ucNewMasterKey[16]; 
  unsigned char ucInputAID[16]; 
}InpCmdVectorStruct;


//=================================================================//
//  Initialize variable for Mifare desfire card read /write 
//

#define MAX_BUFF_SIZE       2000

unsigned char FileSize[4];
unsigned int CardInfoLen; 

//----- Project AMK default 0x00 -------------------
unsigned char AMK[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;

unsigned char CardInfo[MAX_BUFF_SIZE+1]; // Content: “UID;.....”

unsigned char volume = Buzzer_VB;  // default volume for beep
int seclock = 5; // Time for autolock after unlock default 5 second

unsigned int battery_VADC_value;
unsigned int Temp_VADC_value;
BatteryTestData Battery_Test_Data;
char Direction;
//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************

static void BoardInit();
void OffLock(unsigned short timer);
// cavin add 
void Delay_10ms(unsigned short var);
void Un_Lock(void);
void Lock(void);
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX + 1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned long  g_ulIpAddr = 0;
unsigned char tick = 0;
unsigned short lock_timer = 0;
unsigned short LedOnOffFlag = 0;
unsigned char LedCounter[3] = {0}, LedTimes[3] = {0}, Led_Delay[3] = {0};
unsigned char InDataLen = 0;
unsigned char InData[8];
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulTIMERA1Base;
static volatile unsigned long g_ulTimerA1TimerInts = 0;
unsigned short delay_counter = 0;
unsigned char BuzzerTimes = 0, BuzzerCounter = 0, OnOffFlag = 0, Buzzer_Delay;
unsigned short RUN_Time = 50;

// PWM Timer for Buzzer
#define TIMER_INTERVAL_RELOAD  25000 /* =80000000/25000 =  3.2Khz*/  
#define DUTYCYCLE_GRANULARITY   100  //3200/255

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// Cavin
extern unsigned char pSnr[];
extern unsigned char _tt1[];
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************





//*****************************************************************************
//                 GLOBAL Function definition
//*****************************************************************************
extern void moto_unlock(void);
extern unsigned char GPIO_IF_MotorSensStatus(void);
extern void moto_stop(void);
extern unsigned char GPIO_IF_TouchStatus(void);  // Proximity
extern void moto_lock(void);
extern void Timer_IF_InterruptClear(unsigned long ulBase);
extern void  TVccOnOff(char Hi_low);
extern void  MifareReset(char Hi_low);
extern void BattVdectPin(char Hi_low);
extern unsigned char GPIO_IF_KeyCoverSStatus(void);
extern u08_t Nfc_readAID(InpCmdVectorStruct *ps_CmdVector);
extern unsigned int get_adc_voltage(void);
extern void SetAntennaSelectionGPIOs(void);
extern void AntennaSelect(unsigned char ucAntNum);
extern u08_t getPowerSource(void);
extern unsigned char GPIO_IF_PowerSource(void);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************
//*****************************************************************************
// FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//*****************************************************************************

//*****************************************************************************
//
//! \brief Application defined hook (or callback) function - assert
//!
//! \param[in]  pcFile - Pointer to the File Name
//! \param[in]  ulLine - Line Number
//! 
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    //Handle Assert here
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined idle task hook
//! 
//! \param  none
//! 
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void)
{
    //Handle Idle Hook for Profiling, Power Management etc
}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//! 
//! \param  none
//! 
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    //Handle Memory Allocation Errors
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//! 
//! \param  none
//! 
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook( OsiTaskHandle *pxTask,
                                   signed char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}
//*****************************************************************************
//
//! This function gets triggered when HTTP Server receives Application
//! defined GET and POST HTTP Tokens.
//!
//! \param pHttpServerEvent Pointer indicating http server event
//! \param pHttpServerResponse Pointer indicating http server response
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, 
                                  SlHttpServerResponse_t *pSlHttpServerResponse)
{

}

//*****************************************************************************
//
//!  \brief This function handles WLAN events
//!
//!  \param[in] pSlWlanEvent is the event passed to the handler
//!
//!  \return None
//
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
        return;
    }

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//!  \brief This function handles events for IP address 
//!           acquisition via DHCP indication
//!
//!  \param[in] pNetAppEvent is the event passed to the handler
//!
//!  \return None
//
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(!pNetAppEvent)
    {
        return;
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            g_ulIpAddr = pEventData->ip;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                        "Gateway=%d.%d.%d.%d\n\r",

                        SL_IPV4_BYTE(g_ulIpAddr,3),
                        SL_IPV4_BYTE(g_ulIpAddr,2),
                        SL_IPV4_BYTE(g_ulIpAddr,1),
                        SL_IPV4_BYTE(g_ulIpAddr,0),
                        SL_IPV4_BYTE(g_ulGatewayIP,3),
                        SL_IPV4_BYTE(g_ulGatewayIP,2),
                        SL_IPV4_BYTE(g_ulGatewayIP,1),
                        SL_IPV4_BYTE(g_ulGatewayIP,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{

}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************
//****************************************************************************
//
//!  \brief Connecting to a WLAN Accesspoint
//!
//!   This function connects to the required AP (SSID_NAME) with Security
//!   parameters specified in te form of macros at the top of this file
//!
//!   \param[in]              None
//!
//!   \return       status value
//!
//!   \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, \
                                    &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);
    lock_timer = 3000;
    // Wait for WLAN Event
    while(((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) && lock_timer)
    {
        // Toggle LEDs to Indicate Connection Progress
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
      
    }

    if( IS_CONNECTED(g_ulStatus) || IS_IP_ACQUIRED(g_ulStatus) )
    {
        return SUCCESS;
    }
    else
    {
        return FAILURE;
    }
}
//*****************************************************************************
//
//! Disconnect  Disconnects from an Access Point
//!
//! \param  none
//!
//! \return 0 disconnected done, other already disconnected
//
//*****************************************************************************
long Network_IF_DisconnectFromAP()
{
    long lRetVal = 0;

    if (IS_CONNECTED(g_ulStatus))
    {
        lRetVal = sl_WlanDisconnect();
        if(0 == lRetVal)
        {
            // Wait
            while(IS_CONNECTED(g_ulStatus))
            {
    #ifndef SL_PLATFORM_MULTI_THREADED
                  _SlNonOsMainLoopTask();
    #else
                  osi_Sleep(1);
    #endif
            }
            return lRetVal;
        }
        else
        {
            return lRetVal;
        }
    }
    else
    {
        return lRetVal;
    }

}
//*****************************************************************************
//
//! Delay 10 mini second
//!
//! \param  the delay time x 10ms
//!
//! \return none
//!
//*****************************************************************************
void Delay_10ms(unsigned short var)
{
   MAP_UtilsDelay(135000); //????
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs) || defined (gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);
    PRCMCC3200MCUInit();
}

//===========================================================================================   
unsigned char  Polling_Mifare(void)
{

  signed char RetStatus;
  
  RetStatus = PcdRequest_D();  // send REQA// 尋卡  // 尋卡
  if(RetStatus != MI_OK)
    return 0;

  RetStatus = PcdAnticoll() ;  // 防碰撞 讀到序號到*pSnr
  if(RetStatus != MI_OK)
    return 0;

  RetStatus = PcdSelect();   // 選卡
  if(RetStatus != MI_OK)
    return 0;

  memset(&pSnr[4], 0, 4);   // 清後面 4 個bytes
  
  if((_tt1[0] & 0xc0) == 0x40)// && _snr[0]==0x88)  // 如果是 7個bytes 的卡繼續讀後面的
  {
    if((RetStatus = PcdAnticoll2()) != MI_OK)// 防碰撞
      return 0;
    if((RetStatus = PcdSelect2()) != MI_OK)// 鎖定這個序號的卡片
      return 0;
      
    memcpy(InData, &pSnr[1], 7);
    InDataLen=7;//指定卡號長度 7個bytes 
  }
  else
  {
    memcpy(InData, pSnr, 4);
    InDataLen = 4;  //指定卡號長度 4個bytes 
  }
  
  return 1;
} 
//****************************************************************************
//
//!  get Power source 
//!
//! \param non
//! 
//! This function  
//!    1. To get the Power supply source is from battery or AC/DC adapter 
//!
//! \return 
//     1:  use battery power 
//     0:  AC/DC Adapter 
//****************************************************************************
u08_t getPowerSource(void)
{
  
  u08_t PowerSource_Status;
  
  PowerSource_Status=GPIO_IF_PowerSource(); //  1=Battery power  0=AC/DC Adapter
  return PowerSource_Status;
}
//****************************************************************************
//
//! Update the dutycycle of the PWM timer
//!
//! \param ulBase is the base address of the timer to be configured
//! \param ulTimer is the timer to be setup (TIMER_A or  TIMER_B)
//! \param ucLevel translates to duty cycle settings (0:255)
//! 
//! This function  
//!    1. The specified timer is setup to operate as PWM
//!
//! \return None.
//
//****************************************************************************
void UpdateDutyCycle(unsigned long ulBase, unsigned long ulTimer,
                     unsigned char ucLevel)
{
    //
    // Match value is updated to reflect the new dutycycle settings
    //
    MAP_TimerMatchSet(ulBase, ulTimer, (ucLevel * DUTYCYCLE_GRANULARITY));
}

//****************************************************************************
//
//! Setup the timer in PWM mode
//!
//! \param ulBase is the base address of the timer to be configured
//! \param ulTimer is the timer to be setup (TIMER_A or  TIMER_B)
//! \param ulConfig is the timer configuration setting
//! \param ucInvert is to select the inversion of the output
//! 
//! This function  
//!    1. The specified timer is setup to operate as PWM
//!
//! \return None.
//
//****************************************************************************
void SetupTimerPWMMode(unsigned long ulBase, unsigned long ulTimer,
                       unsigned long ulConfig, unsigned char ucInvert)
{
    //
    // Set GPT - Configured Timer in PWM mode.
    //
    MAP_TimerConfigure(ulBase, ulConfig);
    MAP_TimerPrescaleSet(ulBase, ulTimer, 0);
    
    //
    // Inverting the timer output if required
    //
    MAP_TimerControlLevel(ulBase, ulTimer, ucInvert);
    
    //
    // Load value set to ~0.5 ms time period
    //
    MAP_TimerLoadSet(ulBase, ulTimer, TIMER_INTERVAL_RELOAD);
    
    //
    // Match value set so as to output level 0
    //
    MAP_TimerMatchSet(ulBase, ulTimer, TIMER_INTERVAL_RELOAD);
}

//****************************************************************************
//
//! Sets up the identified timers as PWM to drive the peripherals
//!
//! \param none
//! 
//! This function sets up the folowing 
//!    1. TIMERA2 (TIMER B) as Buzzer 
//!
//!
//! \return None.
//
//****************************************************************************
void InitPWMModules()
{
    //
    // Initialization of timers to generate PWM output

    //
    // TIMERA2 (TIMER B)   GPIO 9 --> PWM_5
    //
    SetupTimerPWMMode(TIMERA2_BASE, TIMER_B,
            (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM), 1);
    MAP_TimerEnable(TIMERA2_BASE, TIMER_B);
}

//****************************************************************************
//
//! Disables the timer PWMs
//!
//! \param none
//! 
//! This function disables the timers used
//!
//! \return None.
//
//****************************************************************************
void DeInitPWMModules()
{
    //
    // Disable the peripherals
    //
    MAP_TimerDisable(TIMERA2_BASE, TIMER_B);
    MAP_PRCMPeripheralClkDisable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
}

// to Adjust the buzzer volume by change the Duty value (big :128 mid:32 small:16 mute:0)
//---------------------------------------------------------------------------------------------------
void BUZZER_DutyOn(unsigned char Duty)
{
  UpdateDutyCycle(TIMERA2_BASE, TIMER_B, Duty);
}

void BUZZER_On(void)
{
  BUZZER_DutyOn(volume);
}
//---------------------------------------------------------------------------------------------------
void BUZZER_Off(void)
{
  UpdateDutyCycle(TIMERA2_BASE, TIMER_B, 0);
}


//-------------------------------------------------------------------------
void BuzzerActive(unsigned char buzzer,unsigned char delay)
{
  if(buzzer & 0x80)//buzzer ON  81:50
  {
    BUZZER_Off();
    OnOffFlag = 0;
    if(buzzer & 0x40)//連續嗶聲
    {
      BuzzerTimes = 0x40;
    }
    else
    {
      if((buzzer & 0x3f) == 0)//0x80 持續嗶
      {
         BUZZER_On();
	 OnOffFlag = 1;
	 BuzzerTimes = 0;
      }
      else   // beep 500mS
      {	
        BuzzerTimes = buzzer & 0x3f;  // 0x81
	Buzzer_Delay = delay;         // 50
      }
    } 
  }
  else
  {
    BuzzerTimes = 0;
    BUZZER_Off();
    OnOffFlag = 0;
  }
}	
//---------------------------------------------------------------------------------------------------
void Led_On(unsigned char led)
{
  switch(led)
  {
    case 0:
       GPIO_IF_LedOn(MCU_RED_LED_GPIO);
       break;
     case 1:
       GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
       break;
    case 2:
       GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
       break;
    default:
    break;
  }
}


void Led_Off(unsigned char led)
{
   switch(led)
  {
    case 0:
       GPIO_IF_LedOff(MCU_RED_LED_GPIO);
       break;
    case 1:
       GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
       break;
    case 2:
       GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
       break;
    default:
    break;
  }
}

//----------------------------------------------------------------------------
void Check_BuzzerLed(void)
{
  unsigned char i;
  
  /***  檢查Buzzer On off 的時間 **/
  if(BuzzerTimes)
  {
    if(!BuzzerCounter)
    {
      if(OnOffFlag & 1)
      {
        BUZZER_Off();
        OnOffFlag = 0;
        if(BuzzerTimes < 0x40)//當大於等於16時表示連續嗶,不停止
          BuzzerTimes--;
        BuzzerCounter = 8;
      }
      else
      {
        BUZZER_On();
        OnOffFlag = 1;
        BuzzerCounter = Buzzer_Delay;
      }
    }
  }

  for(i = 0; i < 3; i++)  // 0= Gled ,1= Yled, 2=Rled
  {
    if(LedTimes[i])
    {
      if(!LedCounter[i])  // LED on 的時間計數終了
      {
        if(LedTimes[i] & 0x40)  //LED連續on
        {
           if(LedOnOffFlag & (1 << i))  //find the Turn off LED 
           {
             Led_Off(i);                
             LedOnOffFlag &= ~(1 << i);
             LedCounter[i] = 8;
           }
	   else
	   {
             Led_On(i);
             LedOnOffFlag |= (1 << i);         // 下次LED off    
             LedCounter[i] = Led_Delay[i];   // 設定systick LED on時間
	   }
	}
	else
	{
           if((LedTimes[i] & 0x3f) == 0)  // 單一Beep
           {
              if(LedTimes[i] & 0x80)
              {
                Led_Off(i);
                LedOnOffFlag &= ~(1 << i);  // Clear Flag
              }
              else
              {
                Led_On(i);
                LedOnOffFlag |= (1 << i);
               }
               LedTimes[i] = 0;
            }
            else
            {
              if(LedOnOffFlag & (1 << i))
              {
                Led_Off(i);
                LedOnOffFlag &= ~(1 << i);
                LedCounter[i] = 8;
                if(LedTimes[i] & 0x80)
                {
                  LedTimes[i]--;    
                }
              }
              else
              {
                Led_On(i);
                if(!(LedTimes[i] & 0x80))
                {
                  LedTimes[i]--;    
                }
                LedOnOffFlag |= (1<<i);
                LedCounter[i] = Led_Delay[i];
              }
            }
        }
      }
    }
  }
 }  

void delay_checkbuzzerled(unsigned short delaytime)
{
  delay_counter = delaytime;
  while(delay_counter)
  {
    Check_BuzzerLed();
  }
}

//---------------------------------------------------------------------------------------------------
/*
    啟動馬達順轉，傳動凸輪逆轉無法啟動微動開關(OFF→ON)在OFF位置，
	馬達持續順轉3秒停頓連續5閃紅燈再持續反復轉動，連續持續60秒後馬達停止


	1.	輸入正確密碼後背光即刻熄滅
    2.	連續5嗶短聲(0.1秒)，連續5閃紅燈（0.1秒）(轉動外側把手不放超過開鎖時間持續不放把手)。

    1.	輸入正確密碼後背光即刻熄滅
    2.	1嗶聲（0.5秒）連續閃綠燈(0.1秒)，持續5秒閃綠燈後1嗶聲(0.2秒)
*/
  
//-------------------------------------------------------------------------------

unsigned char getlock(void)
{
  if(GPIO_IF_MotorSensStatus()) //  1=off 0=unlock
    return 0; // off 
   return 1;//unlock
}

void LedActive(unsigned char led,unsigned char buzzer,unsigned char delay)
{
   if(buzzer & 0x80)//buzzer ON
  {
    Led_On(led);
    LedOnOffFlag |= (1 << led);
    LedTimes[led] = buzzer;
    Led_Delay[led] = delay;
    LedCounter[led] = delay;
  }	
  else
  {
    Led_Off(led);
    LedOnOffFlag &= ~(1 << led);
    LedTimes[led] = buzzer;
    Led_Delay[led] = delay;
    LedCounter[led] = delay;
  }
}
//======================================================================
/*
啟動馬達逆轉，傳動插梢順轉至開關(ON-OFF) 0.2秒後馬達停止(上鎖狀態) 如下圖

*/
//======================================================================

void Lock(void)
{  
  moto_lock();
  delay_counter = 15;
  do
  {
      if(getlock())
        break;
      MAP_UtilsDelay(80000); //1mS
  } while(delay_counter);
  MAP_UtilsDelay(80000); //1mS
  moto_stop();
}

void Un_Lock(void)
{
    UART_PRINT("UnLock.....\n\r");
    delay_counter = 50;    //300mS
    moto_unlock(); // -----------
    BuzzerActive(0x81, 30);//1嗶聲（0.3秒）
    LedActive(LED_KEY_G, 0x80, 30);//連續閃綠燈(0.1秒)
    do  // turn on motor  & beep until the motor stop switch on
    {
        if(getlock())
          break;
        Check_BuzzerLed(); 
    } while(BuzzerTimes);
    LedActive(LED_KEY_G, 0, 0);//綠燈off
    do  // turn on motor  & beep until the motor stop switch on
    {
        if(getlock())
          break;
    } while(lock_timer);
    moto_stop();
}     
//---------------------------------------------------------------------------------------------------------------------------------

void UnLock(void)
{
    unsigned char ok=0;
    int lock_status=0;
    
    UART_PRINT("UnLock.....\n\r");
    lock_timer = 100;   
    moto_unlock(); // -----------
    BuzzerActive(0x81, 30);//1嗶聲（0.3秒）
    LedActive(LED_KEY_G, 0xC0, 10);//連續閃綠燈(0.1秒)
    do  // turn on motor  & beep until the motor stop switch on
    {
      if(getlock())
        break;
      Check_BuzzerLed(); 
      lock_status = 1;
    } while(BuzzerTimes);
     
    do  // turn on motor  & beep until the motor stop switch on
    {
      if(getlock())
        break;
      lock_status = 1;
    } while(lock_timer);
    
    moto_stop();
    UART_PRINT("Blinking Green LED for 5 second .....\n\r");
    lock_timer = seclock * 100;          //Seccond *100*10mS;    //default Lock again after unlock for 5second
    LedActive(LED_KEY_G, 0xC0, 10);//連續閃綠燈(0.1秒)
    LedActive(LED_KEY_Y, 0x81, 200);//連續閃 白光 (0.3秒)  
    do
    {
      Check_BuzzerLed();  
    } while(lock_timer);              // delay 5S	
    ok = 1;  // for testing 
    LedActive(LED_KEY_G, 0, 0);//  綠燈off
    if(ok  && lock_status)
    {
      OffLock(5);
    }
    else
    {
      BuzzerActive(0x85, 10);// 連續5嗶短聲(0.1秒)
      LedActive(LED_KEY_R, 0x85, 10);// 連續5閃紅燈（0.1秒）
      do  // turn on  beep until 
      {
        Check_BuzzerLed(); 
      } while(BuzzerTimes);
    }
    LedActive(LED_KEY_G, 0, 0);//  綠燈off
    LedActive(LED_KEY_Y, 0, 0);//  白燈off  
}
//======================================================================
/*
啟動馬達逆轉，傳動插梢順轉至開關(ON-OFF) 0.2秒後馬達停止(上鎖狀態) 如下圖

*/
//======================================================================

void OffLock(unsigned short timer)
{  
   moto_lock();
   delay_counter = 10 + timer;
   do
   {
      if(!getlock())
        break;
   } while(delay_counter);
   
   // ---  20mS  more forward ---
   delay_counter = 2;  //20mS

   do
   {
      MAP_UtilsDelay(100); //1mS 
   } while(delay_counter);
 
   moto_stop();  
}

//---------------------------------------------------------------------------------------------------
void systick_isr(void)   // 10ms tick
{
   unsigned char i;
   
   //
   // Clear the timer interrupt.
   //
   Timer_IF_InterruptClear(g_ulBase);
   if(lock_timer)
      --lock_timer;

   if(tick)
      --tick;

   if(delay_counter)
   {
      --delay_counter;
   }
   
   if(BuzzerCounter)
      --BuzzerCounter;
 
   for(i=0;i<3;i++)
   {
      if(LedCounter[i])
        --LedCounter[i];
   }	
}
//---------------------------------------------------------------------------------------------------
void SysTick_Init(unsigned long mS)
{
    //
    // Base address for first timer
    //
    g_ulBase = TIMERA0_BASE;
    //
    //
    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, systick_isr /*TimerBaseIntHandler*/);
    //
    // Turn on the timers feeding values in mSec
    //
    Timer_IF_Start(g_ulBase, TIMER_A, mS);   // 10mS Preodic
}
//****************************************************************************
//
//! Func :Temprature_ADC
// Return val: The value of Temprature_ADC
//
//****************************************************************************
//
#define NO_OF_SAMPLES 	32   //	128

unsigned int Temperature_ADC(void)
{
      float RetVal;
      unsigned long  uiAdcInputPin = PIN_58;  
      unsigned int   uiChannel = ADC_CH_1;
      unsigned int  uiIndex = 0;
      unsigned long ulSample;      
      unsigned int RETValBuf, RETvalInt;
      unsigned long pulAdcSamples[80];
      //TempControlPin(1);  // Turn on temperature Sensor IC VCC power 
           
#ifdef CC3200_ES_1_2_1
      //
      // Enable ADC clocks.###IMPORTANT###Need to be removed for PG 1.32
      //
      HWREG(GPRCM_BASE + GPRCM_O_ADC_CLK_CONFIG) = 0x00000043;
      HWREG(ADC_BASE + ADC_O_ADC_CTRL) = 0x00000004;
      HWREG(ADC_BASE + ADC_O_ADC_SPARE0) = 0x00000100;
      HWREG(ADC_BASE + ADC_O_ADC_SPARE1) = 0x0355AA00;
#endif
      //
      // Pinmux for the selected ADC input pin
      //
      MAP_PinTypeADC(uiAdcInputPin, PIN_MODE_255/*PIN_MODE_4*/);
      //
      // Configure ADC timer which is used to timestamp the ADC data samples
      //
      MAP_ADCTimerConfig(ADC_BASE, 2^17);
      //
      // Enable ADC timer which is used to timestamp the ADC data samples
      //
      MAP_ADCTimerEnable(ADC_BASE);
      //
      // Enable ADC module
      //
      MAP_ADCEnable(ADC_BASE);
      //
      // Enable ADC channel
      //
      MAP_ADCChannelEnable(ADC_BASE, uiChannel);

      while(uiIndex < NO_OF_SAMPLES + 4)
      {
          if(MAP_ADCFIFOLvlGet(ADC_BASE, uiChannel))
          {
              ulSample = MAP_ADCFIFORead(ADC_BASE, uiChannel);
              pulAdcSamples[uiIndex++] = ulSample;
          }
      }

      MAP_ADCChannelDisable(ADC_BASE, uiChannel);

      //
      // Print out ADC samples
      //
      //TempControlPin(0);  //GPIO_24 Turn off Temperature sensor IC VCC　power  
      RetVal =((pulAdcSamples[NO_OF_SAMPLES] / 4 ) & 0x0FFF) * 1.4 / 4096;
      RETvalInt =((pulAdcSamples[NO_OF_SAMPLES] / 4 ) & 0x0FFF);
      RETValBuf=RETvalInt;
      UART_PRINT("\n\r Temperature Voltage value is %f\n\r",  RetVal); 

      return RETValBuf;
}
//****************************************************************************
//
//! Func :Battery_V_ADC
// Return val: The value of battery voltage 
//
//****************************************************************************
//
//#define NO_OF_SAMPLES 	10   //	128

unsigned int Battery_V_ADC(void)
{
      unsigned long  uiAdcInputPin = PIN_60;  
      unsigned int   uiChannel = ADC_CH_3;
      unsigned int  uiIndex = 0;
      unsigned long ulSample;      
      unsigned int RETValBuf = 0;
      unsigned int RETvalInt = 0;
      unsigned long pulAdcSamples[NO_OF_SAMPLES];
      BattVdectPin(1);  // Turn on Battery voltage detect  C.K.T Tr
           
#ifdef CC3200_ES_1_2_1
      //
      // Enable ADC clocks.###IMPORTANT###Need to be removed for PG 1.32
      //
      HWREG(GPRCM_BASE + GPRCM_O_ADC_CLK_CONFIG) = 0x00000043;
      HWREG(ADC_BASE + ADC_O_ADC_CTRL) = 0x00000004;
      HWREG(ADC_BASE + ADC_O_ADC_SPARE0) = 0x00000100;
      HWREG(ADC_BASE + ADC_O_ADC_SPARE1) = 0x0355AA00;
#endif
      //
      // Pinmux for the selected ADC input pin
      //
      MAP_PinTypeADC(uiAdcInputPin,PIN_MODE_4/*PIN_MODE_255*/);
      //
      // Configure ADC timer which is used to timestamp the ADC data samples
      //
      MAP_ADCTimerConfig(ADC_BASE,2^17);
      //
      // Enable ADC timer which is used to timestamp the ADC data samples
      //
      MAP_ADCTimerEnable(ADC_BASE);
      //
      // Enable ADC module
      //
      MAP_ADCEnable(ADC_BASE);
      //
      // Enable ADC channel
      //
      MAP_ADCChannelEnable(ADC_BASE, uiChannel);

      while(uiIndex < NO_OF_SAMPLES)
      {
          if(MAP_ADCFIFOLvlGet(ADC_BASE, uiChannel))
          {
              ulSample = MAP_ADCFIFORead(ADC_BASE, uiChannel);
              pulAdcSamples[uiIndex++] = ulSample;
          }
      }

      MAP_ADCChannelDisable(ADC_BASE, uiChannel);

      //
      // Print out ADC samples
      // 
      BattVdectPin(0);  //GPIO_28 Turn off Battery voltage detect  C.K.T Tr
        
      for(uiIndex = 0; uiIndex < 16; uiIndex++)
      {
          RETvalInt = RETvalInt + ((pulAdcSamples[NO_OF_SAMPLES - uiIndex - 1] >> 2 ) & 0x0FFF);
      }
      RETValBuf = RETvalInt >> 4;
       
      return RETValBuf;
}



unsigned int get_adc_voltage(void)
{
   unsigned int Batt_val;
   
   Batt_val = Battery_V_ADC();
   return Batt_val;
}

unsigned int get_adc_Temperature(void)
{
   unsigned int Temp_val;
   
   Temp_val = Temperature_ADC();
   return Temp_val;
}

//---------------------------------------------------------------------------------------------------

int Card_Detect(unsigned char *CardInforPtr)
{
    int retVal = 0;  
    int Scan_num = 0;
    InpCmdVectorStruct s_InpCmdVector;
   
    UART_PRINT("Mifare Scaning ...\n\r");
    InDataLen = 0;  
    RUN_Time = 5; 
    while(RUN_Time)
    {
        MifareReset(0);   // Set MFRC522 Reset pin Low 
        MAP_UtilsDelay(80000); //1mS
        MifareReset(1);   // Set MFRC522 Reset pin high 
        MAP_UtilsDelay(80000); //1mS   
        PcdReset();  // Reset PCD   
        PcdAntennaOn();   //NFC Antenna power on
        Scan_num += 1;
        UART_PRINT(" Mifare Polling scan [%d] \n\r",Scan_num);    
        
        if(Polling_Mifare())// if card scaned there are 4 byte card serial no
        {
            UART_PRINT("============================================\n\r");
            UART_PRINT("Read card UID\n\r");     
            UART_PRINT("UID len = %d \n\r",InDataLen);
            UART_PRINT("UID =");
            for(int i = 0; i < InDataLen  ; i++)
            {
                *CardInforPtr++ = InData[i];
                UART_PRINT(" %02x",InData[i]); 
            }
            UART_PRINT("\r\n");
            if( InDataLen == 7)
            {
                *CardInforPtr++ = ';';   
                //  Set Prject AMK   
                for(int i = 0 ; i < 16 ; i++)
                {
                    s_InpCmdVector.ucMasterKey[i] = AMK[i];   // default AMK   
                }
                retVal = Nfc_readAID(&s_InpCmdVector); // 180mS (29bytes)// 
                if(retVal == 0)
                    retVal =- 1;     
            }
            InDataLen = 0;   
            RUN_Time = 1;
        }
        PcdAntennaOff();  
        RUN_Time -= 1;
        Delay_10ms(10);
    }
    return retVal;
 }

void EnterHIBernate()
{
   MAP_PRCMHibernateWakeupSourceEnable(PRCM_HIB_GPIO17);  
   MAP_PRCMHibernateWakeUpGPIOSelect(PRCM_HIB_GPIO17, PRCM_HIB_FALL_EDGE ); // Proximity INT to wake up  
   //
   // powering down SPI Flash to save power 
   //  
   DBG_PRINT("Hibernated !\n\r");
   Delay_10ms(1);
   Utils_SpiFlashDeepPowerDown();
   //
   // Enter HIBernate mode
   //
   MAP_PRCMHibernateEnter();
}

//*****************************************************************************
//
//!  This funtion includes the following steps:
//!  -open a user file for writing
//!  -write "Old MacDonalds" child song 37 times to get just below a 64KB file
//!  -close the user file
//!
//!  /param[out] ulToken : file token
//!  /param[out] lFileHandle : file handle
//!
//!  /return  0:Success, -ve: failure
//
//*****************************************************************************
long WriteFileToDevice(e_WriteType type, unsigned long *ulToken, long *lFileHandle)
{
    long lRetVal = -1;

    if(type == CREATE_WRITE)
    {
      //
      //  create a user file
      //
      lRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                FS_MODE_OPEN_CREATE(1024, \
                          _FS_FILE_OPEN_FLAG_COMMIT | _FS_FILE_PUBLIC_WRITE),
                        ulToken,
                        lFileHandle);
      if(lRetVal < 0)
      {
        //
        // File may already be created
        //
        lRetVal = sl_FsClose(*lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);
      }
      else
      {
        //
        // close the user file
        //
        lRetVal = sl_FsClose(*lFileHandle, 0, 0, 0);
        if (SL_RET_CODE_OK != lRetVal)
        {
            ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
        }
      }
    }
    
    //
    //  open a user file for writing
    //
    lRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                        FS_MODE_OPEN_WRITE, 
                        ulToken,
                        lFileHandle);
    if(lRetVal < 0)
    {
        lRetVal = sl_FsClose(*lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(FILE_OPEN_WRITE_FAILED);
    }
    
    lRetVal = sl_FsWrite(*lFileHandle, 0, Battery_Test_Data.para, sizeof(BatteryTestData));
    if (lRetVal < 0)
    {
        lRetVal = sl_FsClose(*lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(FILE_WRITE_FAILED);
    }
    
    //
    // close the user file
    //
    lRetVal = sl_FsClose(*lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
    }
    
    return SUCCESS;
}

//*****************************************************************************
//
//!  This funtion includes the following steps:
//!    -open the user file for reading
//!    -read the data and compare with the stored buffer
//!    -close the user file
//!
//!  /param[in] ulToken : file token
//!  /param[in] lFileHandle : file handle
//!
//!  /return 0: success, -ve:failure
//
//*****************************************************************************
long ReadFileFromDevice(unsigned long ulToken, long lFileHandle)
{
    long lRetVal = -1;

    //
    // open a user file for reading
    //
    lRetVal = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                        FS_MODE_OPEN_READ,
                        &ulToken,
                        &lFileHandle);
    if(lRetVal < 0)
    {
        lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(FILE_OPEN_READ_FAILED);
    }

    //
    // read the data
    //
    lRetVal = sl_FsRead(lFileHandle, 0, Battery_Test_Data.para, sizeof(BatteryTestData));
    
    if ((lRetVal < 0) || (lRetVal != sizeof(BatteryTestData)))
    {
        lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(FILE_READ_FAILED);
    }
    
    //
    // close the user file
    //
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
    }

    return SUCCESS;
}
//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param[in]    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}
//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}

static long ConnectAP()
{
    long lRetVal = -1;
    
    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0)
    {
      UART_PRINT("Failed to establish connection w/ an AP \n\r");
      BUZZER_DutyOn(Buzzer_VB);
      GPIO_IF_LedOn(MCU_RED_LED_GPIO);  // for testing
      MAP_UtilsDelay(40000000); 
      GPIO_IF_LedOff(MCU_RED_LED_GPIO);  // for testing
      BUZZER_Off();
    }
    else 
    {
      GPIO_IF_LedOff(MCU_ALL_LED_IND);
      BUZZER_DutyOn(Buzzer_VB);
      GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  // for testing
      MAP_UtilsDelay(40000000); 
      GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);  // for testing
      BUZZER_Off();
      UART_PRINT("Connected to AP: %s \n\r",SSID_NAME);
      UART_PRINT("Device IP: %d.%d.%d.%d\n\r\n\r",
                      SL_IPV4_BYTE(g_ulIpAddr,3),
                      SL_IPV4_BYTE(g_ulIpAddr,2),
                      SL_IPV4_BYTE(g_ulIpAddr,1),
                      SL_IPV4_BYTE(g_ulIpAddr,0));
    }

    return lRetVal;
}

void WlanStation()
{
    long lRetVal = -1;
    
    InitializeAppVariables();

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
        {
            UART_PRINT("Failed to configure the device in its default state\n\r");
        }

        LOOP_FOREVER();
    }

    UART_PRINT("Device is configured in default state \n\r");

    //
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Device started as STATION \n\r");
}
//****************************************************************************
//
//! \brief Start simplelink, connect to the ap and run the ping test
//!
//! This function starts the simplelink, connect to the ap and start the ping
//! test on the default gateway for the ap
//!
//! \param[in]  pvParameters - Pointer to the list of parameters that 
//!             can bepassed to the task while creating it
//!
//! \return  None
//
//****************************************************************************
void WlanStationMode( void *pvParameters )
{
    long lRetVal = -1;
    
    while(1)
    {
    WlanStation();
    lRetVal = ConnectAP();
    Network_IF_DisconnectFromAP();

    //
    // power off the network processor
    //
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    }
}
//****************************************************************************
//
//! \brief Start simplelink, connect to the ap and run the ping test
//!
//! This function starts the simplelink, connect to the ap and start the ping
//! test on the default gateway for the ap
//!
//! \param[in]  pvParameters - Pointer to the list of parameters that 
//!             can bepassed to the task while creating it
//!
//! \return  None
//
//****************************************************************************
void LockMode( void *pvParameters )
{
    long lFileHandle;
    //unsigned char policyVal;
    unsigned long ulToken;
    long lRetVal = -1;
   
    // Check the wakeup source. If first itme power on entry or wakeup from HIB
    //
    if(MAP_PRCMSysResetCauseGet() == 0)    //------------------------->   // Wake up From power On
    {
       UART_PRINT("n\r\\n\r===============================================\n\r");
       UART_PRINT("\n\rHIB: Wake up on Power ON\n\r");
       Delay_10ms(10);
       if(getPowerSource())
          UART_PRINT( "\n\r------------Battery power system . ----------------\n\r");
       else
          UART_PRINT( "\n\r\n\r------------Adapter  power system . ----------------\n\r");
       
       GPIO_IF_LedOn(MCU_ALL_LED_IND); 
       BuzzerActive(0x81, 30);//1嗶聲（0.3秒）
       do  // turn on motor  & beep until the motor stop switch on
       {         
          Check_BuzzerLed();  
       }while(BuzzerTimes);
       
       //===============  LED Testing =====================
       GPIO_IF_LedOff(MCU_ALL_LED_IND); 

       GPIO_IF_LedOn(MCU_RED_LED_GPIO);  // for testing
       delay_counter = 50;
       while(delay_counter);
       GPIO_IF_LedOff(MCU_RED_LED_GPIO);  // for testing
   
       GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);  // for testing
       delay_counter = 50;
       while(delay_counter);
       GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);  // for testing
            
       GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);  // for testing
       delay_counter = 50;
       while(delay_counter);
       GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);  // for testing
         
       //===============  Lock motor reset  =====================
       if(getlock())  // if UnLock state 
       {
          OffLock(5);    // then Lock again 
       }
       else
       {
          moto_unlock();
          lock_timer = 200;  // 80*10ms 
          do  // turn on motor  & beep until the motor stop switch on
          {
            if(getlock())
              break;
          } while(lock_timer);
          moto_stop();
          lock_timer = 50;          //Seccond *100*10mS;    //default Lock again after unlock for 5second
          LedActive(LED_KEY_G, 0xC0, 10);//連續閃綠燈(0.1秒)
          LedActive(LED_KEY_Y, 0xC0, 30);//連續閃 白光 (0.3秒)
          BuzzerActive(0x81, 30);//1嗶聲（0.3秒）
          do
          {
            Check_BuzzerLed();  
          } while(lock_timer);              // delay 500ms	
          LedActive(LED_KEY_G, 0, 0);//  綠燈off
          LedActive(LED_KEY_Y, 0, 0);//  白燈off           
          OffLock(5);
       }
       Battery_Test_Data.data = 1;
       WlanStation();
       WriteFileToDevice(CREATE_WRITE, &ulToken, &lFileHandle);
       lRetVal = ConnectAP();
       MifareReset(0);  //Low for power saving
       Network_IF_DisconnectFromAP();

       //
       // power off the network processor
       //
       lRetVal = sl_Stop(SL_STOP_TIMEOUT);
       
    }
    else if(MAP_PRCMSysResetCauseGet() == PRCM_HIB_EXIT)  //----->  Wake up From GPIO or SLOW clk
    { 
       UART_PRINT("\n\r\n\r******============================================******\n\r");
       UART_PRINT("\n\rHIB: Woken up from Hibernate\n\r");
       WlanStation();
       if(GPIO_IF_TouchStatus() == 0) //  -------->// Wake up from Proximity (Low active)
       {
          UART_PRINT("HIB2: wakeup from Proximity...\n\r");
          //-----------------Mifare card read Testing Loop--------------------------------------
          AntennaSelect(1);           // Select outdoor ANT  test
          UART_PRINT( "\n\r\n\r======================================================== \n\r");
          UART_PRINT( "starting the procedures of locking/unlocking emulation \n\r");
          Delay_10ms(10);
          battery_VADC_value = get_adc_voltage();
          ReadFileFromDevice(ulToken, lFileHandle);
          UART_PRINT("\n\r Before Battery ADC_vaule = %04X... \n\r ",battery_VADC_value); 
          if(getlock())  // if UnLock state
          {
            UART_PRINT("unlock state\n\r");
            MifareReset(0);  //Low for power saving
            OffLock(5);
            Battery_Test_Data.data ++;
            WriteFileToDevice(EXISTING_WRITE, &ulToken, &lFileHandle);
          }
          else
          {
            UART_PRINT("lock state\n\r");
            MifareReset(0);  //Low for power saving
            Un_Lock(); 
            /*
            switch(Card_Detect(&CardInfo[0]))
            {
              case 0:
                MifareReset(0);  //Low for power saving
                UART_PRINT( "\n\r\n\r======================\n\r");
                UART_PRINT(         "=   Card not exist ! =\n\r");
                UART_PRINT(         "======================\n\r");
              break;
                   
              case 1:
                MifareReset(0);  //Low for power saving
                Un_Lock(); 
                UART_PRINT( "\n\r\n\r======================\n\r");
                UART_PRINT(         "= Read Card Success !=\n\r");
                UART_PRINT(         "======================\n\r");
                Battery_Test_Data.data ++;
                WriteFileToDevice(EXISTING_WRITE, &ulToken, &lFileHandle);
              break;
                   
              case -1:
                MifareReset(0);  //Low for power saving
                UART_PRINT( "\n\r\n\r======================\n\r");
                UART_PRINT(         "=   Read card error ! =\n\r");
                UART_PRINT(         "=======================\n\r");
              break;
                   
              default:
                MifareReset(0);  //Low for power saving
              break; 
            }
            */
          }
            
          Delay_10ms(10);
          battery_VADC_value = get_adc_voltage();
          UART_PRINT("\n\r Count number = %u\n\r ",Battery_Test_Data.data);
          UART_PRINT("\n After Battery ADC_vaule=%04X... \n\r ",battery_VADC_value);
          Delay_10ms(10);
        
        }  
  
        lRetVal = ConnectAP();
        MifareReset(0);  //Low for power saving
        Network_IF_DisconnectFromAP();
       
        //
        // power off the network processor
        //
        sl_Stop(0xff);        
    }
    EnterHIBernate();
}
//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************

void main()
{
    long lRetVal = -1;
    
    //
    // Board Initialization
    //
    BoardInit();
    //
    // Initialize the uDMA
    //
    UDMAInit();
    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();
    SetAntennaSelectionGPIOs();
    //
    // Configuring UART
    //
    InitTerm();
    // Initialize PWM for Buzzer
    InitPWMModules();
    // Configure All GPIO
    //
    GPIO_IF_Configure(); 
    // Initialize All GPIO off
    GPIO_IF_I2COff(MCU_ALL_LED_IND);
    GPIO_IF_LedOff(MCU_ALL_LED_IND); 
    GPIO_IF_MOTOROff();
     
    // Reset Proximity & Mifare
    TVccOnOff(1);      // Turn Off Proximity power     
     
    MifareReset(0);   // Set MFRC522 Reset pin Low   
    MAP_UtilsDelay(80000); //1mS
    MifareReset(1);   // Set MFRC522 Reset pin high     
    MAP_UtilsDelay(80000); //1mS
       
    SysTick_Init(10);  //10mS Periodic
    AntennaSelect(2);  // Select outdoor ANT  : 1 = outdoor, 2 = indoor
    I2C_IF_Open(I2C_MASTER_MODE_STD);
    
    InitializeAppVariables();
    
    //
    // reset all network policies
    //  
   // sl_WlanPolicySet(SL_POLICY_CONNECTION,
   //                 SL_CONNECTION_POLICY(0, 0, 0, 0, 0),
   //                 &policyVal,
    //                1 /*PolicyValLen*/);
     
    // Reset Proximity & Mifare
    // 
    MifareReset(0);   // Set MFRC522 Reset pin Low   
    MAP_UtilsDelay(80000); //1mS
    MifareReset(1);   // Set MFRC522 Reset pin high     
    MAP_UtilsDelay(80000); //1mS
    SysTick_Init(10);  //10mS Periodic
    
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_STD);
    
    //
    // Start the SimpleLink Host
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    
    //
    // Start the WlanStationMode task
    //
    lRetVal = osi_TaskCreate( WlanStationMode, \
                                (const signed char*)"Wlan Station Task", \
                                OSI_STACK_SIZE, NULL, 1, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
     
    //
    // Start the task scheduler
    //
    osi_start();
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
