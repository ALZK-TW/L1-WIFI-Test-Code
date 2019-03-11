/////////////////////////////////////////////////////////////////////
//	函數原型
/////////////////////////////////////////////////////////////////////
void PcdReset(void);
void PcdAntennaOn(void);
void PcdAntennaOff(void);

void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData);
void WriteRawRC(unsigned char Address,unsigned char value);
unsigned char ReadRawRC(unsigned char Address);
void SetBitMask(unsigned char reg,unsigned char mask);
void ClearBitMask(unsigned char reg,unsigned char mask);
signed char  Polling_Mifare_1(unsigned char *buff);
//signed char  Polling_Mifare_1(void);
void PcdHalt_1(void);
signed char PcdComMF522(unsigned char Command,unsigned char *pInData,unsigned char InLenByte);


signed char PcdRequest(void);
signed char PcdAnticoll(void);
signed char PcdSelect(void);
signed char PcdAnticoll2(void);
signed char PcdSelect2(void);
signed char PcdAnticoll3(void);
signed char PcdSelect3(void);
signed char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey);

//signed char PcdRequest_D(unsigned char req_code,unsigned char *pTagType);
signed char PcdRequest_D(void);//(unsigned char req_code,unsigned char *pTagType); //jimmy modified
signed char PcdAnticoll_D(unsigned char AnticollType,unsigned char *Snr);
signed char PcdSelect_D(unsigned char AnticollType,unsigned char *Snr);
signed char PcdAuthState_D(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *Snr);

signed char PcdWrite(unsigned char addr,unsigned char *pData);
signed char PcdRead(unsigned char addr,unsigned char *buff);
void PcdHalt(void);

//signed char PiccRequestATS(void);
//signed char PiccPPSRequest(void);
//signed char PiccSelectADF(void);
//signed char PiccGetRandom(void);
//signed char  PiccReadCPU(unsigned char *buff);
void CPU_Halt(void);
void PcdRequest_1(void);
signed char MifareResponse(void);
signed char PiccGetFx(unsigned char *inbuff,unsigned char *outbuff);
signed char PiccSelectADF(void);
signed char PiccRequestATS(void);
signed char PiccPPSRequest(void);
void mifare_io_powerdown(void);

//Jimmy Add-begin
#include "iso14443a.h"

signed char  Iso14443aLayer4(void);
u08_t  Aes_authenticate(u08_t * pui8SessionKey, u08_t * pui8RndA, u08_t * pui8Key);
void Nfc_setAesKey(const u08_t * pui8AESKey, u08_t ui8AESKeyLength);
void Nfc_runAesAuth(void);
//Jimmy Add-end
/////////////////////////////////////////////////////////////////////
//	MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00               //取消當前命令
#define PCD_AUTHENT           0x0E               //驗證密鑰
#define PCD_RECEIVE           0x08               //接收數據
#define PCD_TRANSMIT          0x04               //發送數據
#define PCD_TRANSCEIVE        0x0C               //發送並接收數據
#define PCD_RESETPHASE        0x0F               //復位
#define PCD_CALCCRC           0x03               //CRC計算

/////////////////////////////////////////////////////////////////////
//	Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //尋天線區內未進入休眠狀態
#define PICC_REQALL           0x52               //尋天線區內全部卡
#define PICC_ANTICOLL1        0x93               //防衝撞
#define PICC_ANTICOLL2        0x95               //防衝撞
#define PICC_ANTICOLL3        0x97               //防衝撞
#define PICC_AUTHENT1A        0x60               //驗證A密鑰
#define PICC_AUTHENT1B        0x61               //驗證B密鑰
#define PICC_READ             0x30               //讀塊
#define PICC_WRITE            0xA0               //寫塊
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //調塊數據到緩衝區
#define PICC_TRANSFER         0xB0               //保存緩衝區中數據
#define PICC_HALT             0x50               //休眠

/////////////////////////////////////////////////////////////////////
//	MF522 FIFO長度定義
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte

/////////////////////////////////////////////////////////////////////
//	MF522寄存器定義
/////////////////////////////////////////////////////////////////////
// PAGE 0
#define     RFU00                 0x00
#define     CommandReg            0x01
#define     ComIEnReg             0x02
#define     DivlEnReg             0x03
#define     ComIrqReg             0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2
#define     RFU20                 0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     RFU3C                 0x3C
#define     RFU3D                 0x3D
#define     RFU3E                 0x3E
#define     RFU3F		  0x3F

/////////////////////////////////////////////////////////////////////
//	和MF522通訊時返回的錯誤代碼
/////////////////////////////////////////////////////////////////////
#define MI_OK                          0
#define MI_NOTAGERR                    (-1)
#define MI_ERR                         (-2)
#define STATUS_PPS_ERROR		(-104)
#define MI_BITCOUNTERR                  (-11)
#define MI_EMPTY                        (-3)


//====================================================================
//Jimmy Add-begin

//===============================================================
//		Global Variables
//===============================================================

//static unsigned char pui8SessionKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//for testing AES crypt
//static unsigned char pui8TestSessionKey[16] = {0x04, 0xBC, 0x99, 0xA8, 0x1D, 0xB7, 0x29, 0x3F, 0xAA, 0x86, 0xCA, 0x22, 0x5E, 0xCD, 0x76, 0x60};

//static unsigned char puiTestData[16] = {0x04, 0xBC, 0x99, 0xA8, 0x1D, 0xB7, 0x29, 0x3F, 0xAA, 0x86, 0xCA, 0x22, 0x5E, 0xCD, 0x76, 0x60};

// This is the Random Number A (RndA) that will be used during the AES authentication sequence.
// It is hard coded in this example, however a Random Number Generator (RNG) could be used to generate the number instead.
//static unsigned char pui8RndA[16] = {0x79, 0xd4, 0x66, 0x29,0xf7,0xe1,0x12,0xc3,0x79, 0xd4, 0x66, 0x29,0xf7,0xe1,0x12,0xc3};

// This is the AES key that will be used for the AES authentication process. Modify this key with the Nfc_setAESKey() function.
//static unsigned char pui8AESkey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//static unsigned char pui8AESkey[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
//static u08_t IV[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//Card Master Key
//const static u08_t CustomKey1[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Application Master Key
//const static u08_t CustomKey2[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
//const static u08_t CustomKey3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF};
const static u08_t NewMasterKey[16] = {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF};