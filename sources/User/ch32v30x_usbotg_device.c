/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbotg_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the USBOTG firmware functions.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32v30x_usbotg_device.h"
#include <rtthread.h>

/* Global define */
/* OTH */
#define pMySetupReqPakHD        ((PUSB_SETUP_REQ)EP0_DatabufHD)
#define RepDescSize             62
#define DevEP0SIZE              64
#define PID_OUT                 0
#define PID_SOF                 1
#define PID_IN                  2
#define PID_SETUP               3


/******************************************************************************/
/* ȫ�ֱ��� */
/* Endpoint Buffer */
__attribute__ ((aligned(4))) UINT8 EP0_DatabufHD[64]; //ep0(64)
__attribute__ ((aligned(4))) UINT8 EP1_DatabufHD[64+64];  //ep1_out(64)+ep1_in(64)
__attribute__ ((aligned(4))) UINT8 EP2_DatabufHD[64+64];  //ep2_out(64)+ep2_in(64)
__attribute__ ((aligned(4))) UINT8 EP3_DatabufHD[64+64];  //ep3_out(64)+ep3_in(64)
__attribute__ ((aligned(4))) UINT8 EP4_DatabufHD[64+64];  //ep4_out(64)+ep4_in(64)
__attribute__ ((aligned(4))) UINT8 EP5_DatabufHD[64+64];  //ep5_out(64)+ep5_in(64)
__attribute__ ((aligned(4))) UINT8 EP6_DatabufHD[64+64];  //ep6_out(64)+ep6_in(64)
__attribute__ ((aligned(4))) UINT8 EP7_DatabufHD[64+64];  //ep7_out(64)+ep7_in(64)

PUINT8  pEP0_RAM_Addr;                       //ep0(64)
PUINT8  pEP1_RAM_Addr;                       //ep1_out(64)+ep1_in(64)
PUINT8  pEP2_RAM_Addr;                       //ep2_out(64)+ep2_in(64)
PUINT8  pEP3_RAM_Addr;                       //ep3_out(64)+ep3_in(64)
PUINT8  pEP4_RAM_Addr;                       //ep4_out(64)+ep4_in(64)
PUINT8  pEP5_RAM_Addr;                       //ep5_out(64)+ep5_in(64)
PUINT8  pEP6_RAM_Addr;                       //ep6_out(64)+ep6_in(64)
PUINT8  pEP7_RAM_Addr;                       //ep7_out(64)+ep7_in(64)


const UINT8 *pDescr;
volatile UINT8  USBHD_Dev_SetupReqCode = 0xFF;                                  /* USB2.0�����豸Setup�������� */
volatile UINT16 USBHD_Dev_SetupReqLen = 0x00;                                   /* USB2.0�����豸Setup������ */
volatile UINT8  USBHD_Dev_SetupReqValueH = 0x00;                                /* USB2.0�����豸Setup��Value���ֽ� */
volatile UINT8  USBHD_Dev_Config = 0x00;                                        /* USB2.0�����豸����ֵ */
volatile UINT8  USBHD_Dev_Address = 0x00;                                       /* USB2.0�����豸��ֵַ */
volatile UINT8  USBHD_Dev_SleepStatus = 0x00;                                   /* USB2.0�����豸˯��״̬ */
volatile UINT8  USBHD_Dev_EnumStatus = 0x00;                                    /* USB2.0�����豸ö��״̬ */
volatile UINT8  USBHD_Dev_Endp0_Tog = 0x01;                                     /* USB2.0�����豸�˵�0ͬ����־ */
volatile UINT8  USBHD_Dev_Speed = 0x01;                                         /* USB2.0�����豸�ٶ� */

volatile UINT16 USBHD_Endp1_Up_Flag = 0x00;                                     /* USB2.0�����豸�˵�1�����ϴ�״̬: 0:����; 1:�����ϴ�; */
volatile UINT8  USBHD_Endp1_Down_Flag = 0x00;                                   /* USB2.0�����豸�˵�1�´��ɹ���־ */
volatile UINT8  USBHD_Endp1_Down_Len = 0x00;                                    /* USB2.0�����豸�˵�1�´����� */
volatile BOOL   USBHD_Endp1_T_Tog = 0;                                          /* USB2.0�����豸�˵�1����togλ��ת */
volatile BOOL   USBHD_Endp1_R_Tog = 0;

volatile UINT16 USBHD_Endp2_Up_Flag = 0x00;                                     /* USB2.0�����豸�˵�2�����ϴ�״̬: 0:����; 1:�����ϴ�; */
volatile UINT16 USBHD_Endp2_Up_LoadPtr = 0x00;                                  /* USB2.0�����豸�˵�2�����ϴ�װ��ƫ�� */
volatile UINT8  USBHD_Endp2_Down_Flag = 0x00;                                   /* USB2.0�����豸�˵�2�´��ɹ���־ */

volatile UINT32V Endp2_send_seq=0x00;
volatile UINT8   DevConfig;
volatile UINT8   SetupReqCode;
volatile UINT16  SetupReqLen;

/******************************************************************************/
/* Device Descriptor */

const UINT8  MyDevDescrHD[] = {
    0x12, 0x01, //length = 18 bytes, device descriptor= 0x01
    0x00, 0x02, //bcdusb (2 bytes)
    0x00, 0x00, //device class,subdevice class
    0x00, DevEP0SIZE,//device protocol,max packet size
    0x86, 0x1A, //id vendor(2 bytes)
    0xE1, 0xE6, //idproduct (2 bytes)
    0x00, 0x02, //bcd device (2 bytes)
    0x01, 0x02, //index manufacturer descriptor,index  product
    0x00, 0x01, //index serial number,num configuration
};

/*
 Logitech rumble pad 0x9 0x2 0x29 0x0 0x1 0x1 0x0 0x80 0xFA 0x9 0x4 0x0 0x0 0x2 0x3 0x0 0x0 0x0 0x9 0x21 0x10 0x1 0x0 0x1 0x22 0x77 0x0 0x7 0x5 0x81 0x3 0x8 0x0 0xA 0x7 0x5 0x1 0x3 0x8 0x0 0xA
*/


/* Configuration Descriptor */
const UINT8  MyCfgDescrHD[] =
{
    0x09, //Size 9 bytes
    0x02, //Configuration descriptor 0x02
    0x22,0x00, // Total length in byte returned sizeof(MyCfgDescrHD)
    0x01, // Number of interface
    0x01, // Configuration value 
    0x00, //Index of String Descriptor describing this configuration
    0x80, //Bitmap 0x80 = usb bus powered
    0x32, //Maximum power consuption in mA*2

    0x09, //Size 9 bytes
    0x04, //Interface descriptor 0x04
    0x00,// Interface number
    0x00,// Alternate settings
    0x01,// Number of endpoint
    0x03,// Interface class HID = 0x03
    0x00, // Subclass
    0x00, // Interface protocol
    0x00, // Index of String Descriptor Describing this interface

    0x09,//Size 9
    0x21, //HID descriptor
    0x10,0x01, // HID number
    0x00, //Country code
    0x01, //Num descriptor
    0x22, //Descriptor type: data descriptor
    0x30,0x00, //Length sizeof(MyReportDescHD) 

    
    // 0x07,
    // 0x05,
    // 0x01,// Endpoint address Direction in, Endpoint 1
    // 0x03,
    // 0x04,0x00,
    // 0x0A,
   
    0x07,// Size 7 bytes
    0x05,// Endpoint descriptor
    0x81,// Endpoint address Direction IN, Endpoint 1
    0x03,// Attribute Transfert type: Interrupt 
    0x04,0x00, //Max packet size 
    0x0A //Interval for polling endpoint data transfers in frame time: 10 ms

    // 0x09, 0x02, 0x2E, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
    // 0x09, 0x04, 0x00, 0x00, 0x04, 0xFF, 0x80, 0x55, 0x00,
    // 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
    // 0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, //endpoint
    // 0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
    // 0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
};

/* USB���������� */
const UINT8  MyReportDescHD[ ] =
{
    0x05, 0x01, //USAGE_PAGE generic desktop
    0x09, 0x04, //USAGE joystick
    0xA1, 0x01, //Collection application
  //  0x85, 0x01, // REPORT_ID = 1
    0xA1, 0x02, // COLLECTION logical was A2
    0x15, 0x00, //LOGICAL_MINIMUM(0)
    0x26,0xFF,0x0F,//LOGICAL_MAXIMUM(4096)
    0x35, 0x00,//     PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x0F, //PHYSICAL_MAXIMUM (4096)
    0x75, 0x10,                    //     REPORT_SIZE //Each axis number is a 16 bit number 
    0x95, 0x02,                    //     REPORT_COUNT (2) 2*16 bits value
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    //0x09, 0x32, //     USAGE (Z)
//    0x09, 0x33, //USAGE(RX)
//    0x09, 0x34, //USAGE(RY)
//    0x09, 0x35  //USAGE(RZ)
   
   
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x25,0x01,        //Logical maximum: 1 
    0x45,0x01,        //Physical maximum: 1   
    0x75,0x01,        //Report size: 1   
    0x95,0x08,		//Report count: Button count 
    0x5,0x9,		//	Usage Page: Button 
    0x19,0x01, 		//Usage minimum: button 1
    0x29,0x08,		//Usage maximum: button Button count 
    0x81,0x02, 		    //Input: data


  //  0x05, 0x07,
   0xC0,// END_COLLECTION
   0xC0
};

/* Language Descriptor */
const UINT8  MyLangDescrHD[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufactor Descriptor */
const UINT8  MyManuInfoHD[] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

/* Product Information */
const UINT8  MyProdInfoHD[] =
{
    0x12, 0x03, 'J', 0, 'o', 0, 'y', 0, 's', 0, 't', 0, 'i',0 , 'c', 0, 'k',0
};

/* USB���к��ַ��������� */
const UINT8  MySerNumInfoHD[] =
{
    /* 0123456789 */
    22,03,48,0,49,0,50,0,51,0,52,0,53,0,54,0,55,0,56,0,57,0
};

/* USB�豸�޶������� */
const UINT8 MyUSBQUADescHD[] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00,
};

/* USBȫ��ģʽ,�����ٶ����������� */
UINT8 TAB_USB_FS_OSC_DESC[sizeof(MyCfgDescrHD)] =
{
    0x09, 0x07,                                                                 /* ��������ͨ���������� */
};


void OTG_FS_IRQHandler(void) __attribute__((naked));
void OTG_FS_IRQHandler_impl(void);// __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      USBOTG_FS_DeviceInit
 *
 * @brief   Initializes USB device.
 *
 * @return  none
 */
void USBDeviceInit( void )
{
    USBOTG_FS->BASE_CTRL = 0x00;

    USBOTG_FS->UEP4_1_MOD = USBHD_UEP4_RX_EN|USBHD_UEP4_TX_EN|USBHD_UEP1_RX_EN|USBHD_UEP1_TX_EN;
    USBOTG_FS->UEP2_3_MOD = USBHD_UEP2_RX_EN|USBHD_UEP2_TX_EN|USBHD_UEP3_RX_EN|USBHD_UEP3_TX_EN;
    USBOTG_FS->UEP5_6_MOD = USBHD_UEP5_RX_EN|USBHD_UEP5_TX_EN|USBHD_UEP6_RX_EN|USBHD_UEP6_TX_EN;
    USBOTG_FS->UEP7_MOD   = USBHD_UEP7_RX_EN|USBHD_UEP7_TX_EN;

    USBOTG_FS->UEP0_DMA = (UINT32)pEP0_RAM_Addr;
    USBOTG_FS->UEP1_DMA = (UINT32)pEP1_RAM_Addr;
    USBOTG_FS->UEP2_DMA = (UINT32)pEP2_RAM_Addr;
    USBOTG_FS->UEP3_DMA = (UINT32)pEP3_RAM_Addr;
    USBOTG_FS->UEP4_DMA = (UINT32)pEP4_RAM_Addr;
    USBOTG_FS->UEP5_DMA = (UINT32)pEP5_RAM_Addr;
    USBOTG_FS->UEP6_DMA = (UINT32)pEP6_RAM_Addr;
    USBOTG_FS->UEP7_DMA = (UINT32)pEP7_RAM_Addr;

    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP1_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP2_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP3_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP4_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP5_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP6_RX_CTRL = USBHD_UEP_R_RES_ACK;
    USBOTG_FS->UEP7_RX_CTRL = USBHD_UEP_R_RES_ACK;

    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
    USBOTG_FS->UEP1_TX_LEN = 8;
    USBOTG_FS->UEP2_TX_LEN = 8;
    USBOTG_FS->UEP3_TX_LEN = 8;
    USBOTG_FS->UEP4_TX_LEN = 8;
    USBOTG_FS->UEP5_TX_LEN = 8;
    USBOTG_FS->UEP6_TX_LEN = 8;
    USBOTG_FS->UEP7_TX_LEN = 8;

    USBOTG_FS->UEP1_TX_CTRL = USBHD_UEP_T_RES_ACK;
    USBOTG_FS->UEP2_TX_CTRL = USBHD_UEP_T_RES_ACK|USBHD_UEP_AUTO_TOG;
    USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_ACK|USBHD_UEP_AUTO_TOG;
    USBOTG_FS->UEP4_TX_CTRL = USBHD_UEP_T_RES_ACK|USBHD_UEP_AUTO_TOG;
    USBOTG_FS->UEP5_TX_CTRL = USBHD_UEP_T_RES_ACK|USBHD_UEP_AUTO_TOG;
    USBOTG_FS->UEP6_TX_CTRL = USBHD_UEP_T_RES_ACK|USBHD_UEP_AUTO_TOG;
    USBOTG_FS->UEP7_TX_CTRL = USBHD_UEP_T_RES_ACK|USBHD_UEP_AUTO_TOG;

    USBOTG_FS->INT_FG = 0xFF;
    USBOTG_FS->INT_EN = USBHD_UIE_SUSPEND | USBHD_UIE_BUS_RST | USBHD_UIE_TRANSFER;
    USBOTG_FS->DEV_ADDR = 0x00;

    USBOTG_FS->BASE_CTRL = USBHD_UC_DEV_PU_EN | USBHD_UC_INT_BUSY | USBHD_UC_DMA_EN;
    USBOTG_FS->UDEV_CTRL = USBHD_UD_PD_DIS|USBHD_UD_PORT_EN;
}

/*********************************************************************
 * @fn      USBOTG_RCC_Init
 *
 * @brief   Initializes the usbotg clock configuration.
 *
 * @return  none
 */
void USBOTG_RCC_Init(void)
{
#ifdef CH32V30x_D8C
    RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
    RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
    RCC_USBHSConfig( RCC_USBPLL_Div2 );
    RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
    RCC_USBHSPHYPLLALIVEcmd( ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );

#else
    RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLCLK_Div1);;

#endif

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_OTG_FS, ENABLE );
}

/*********************************************************************
 * @fn      USBOTG_Init
 *
 * @brief   Initializes the USBOTG full speed device.
 *
 * @return  none
 */
void USBOTG_Init( void )
{
    /* �˵㻺������ʼ�� */
    pEP0_RAM_Addr = EP0_DatabufHD;
    pEP1_RAM_Addr = EP1_DatabufHD;
    pEP2_RAM_Addr = EP2_DatabufHD;
    pEP3_RAM_Addr = EP3_DatabufHD;
    pEP4_RAM_Addr = EP4_DatabufHD;
    pEP5_RAM_Addr = EP5_DatabufHD;
    pEP6_RAM_Addr = EP6_DatabufHD;
    pEP7_RAM_Addr = EP7_DatabufHD;
    /* ʹ��usbʱ�� */
    USBOTG_RCC_Init( );
    rt_thread_mdelay(1);                                              
    //Delay_Us(100);
    /* usb�豸��ʼ�� */
    USBDeviceInit( );
    EXTEN->EXTEN_CTR |= EXTEN_USBD_PU_EN;
    /* ʹ��usb�ж� */
    NVIC_EnableIRQ( OTG_FS_IRQn );
}

/*********************************************************************
 * @fn      OTG_FS_IRQHandler
 *
 * @brief   This function handles OTG_FS exception.
 *
 * @return  none
 */

void OTG_FS_IRQHandler(void)
{
  __asm volatile ("call OTG_FS_IRQHandler_impl; mret");
}

void OTG_FS_IRQHandler_impl( void )
{
    UINT8  len, chtype;
    UINT8  intflag, errflag = 0;

    intflag = USBOTG_FS->INT_FG;

    if( intflag & USBHD_UIF_TRANSFER )
    {
        switch ( USBOTG_FS->INT_ST & USBHD_UIS_TOKEN_MASK )
        {
            /* SETUP������ */
            case USBHD_UIS_TOKEN_SETUP:
#if 0
                /* ��ӡ��ǰUsbsetup����  */
                printf( "Setup Req :\n" );
                printf( "%02X ", pSetupReqPakHD->bRequestType );
                printf( "%02X ", pSetupReqPakHD->bRequest );
                printf( "%04X ", pSetupReqPakHD->wValue );
                printf( "%04X ", pSetupReqPakHD->wIndex );
                printf( "%04X ", pSetupReqPakHD->wLength );
                printf( "\n" );

#endif

                USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_TOG|USBHD_UEP_T_RES_NAK;
                USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_TOG|USBHD_UEP_R_RES_ACK;
                SetupReqLen  = pSetupReqPakHD->wLength;
                SetupReqCode = pSetupReqPakHD->bRequest;
                chtype = pSetupReqPakHD->bRequestType;
                len = 0;
                errflag = 0;
                /* �жϵ�ǰ�Ǳ�׼���������������� */
                if ( ( pSetupReqPakHD->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    /* ��������,��������,���������� */
                    if( pSetupReqPakHD->bRequestType & 0x40 )
                    {
                        /* �������� */
                        switch( pSetupReqPakHD->bRequest )
                        {
                            default:
                                errflag = 0xFF;/* ����ʧ�� */
                                break;
                        }
                    }
                    else if( pSetupReqPakHD->bRequestType & 0x20 )
                    {
                        /* HID������ */
                        switch( pSetupReqPakHD->bRequest )
                        {
                        case 0x01: //GetReport
                            break;
                        case 0x02: //GetIdle
                            break;
                        case 0x03: //GetProtocol
                            break;
                        case 0x09: //SetReport
                            break;
                        case 0x0A: //SetIdle
                            break;
                        case 0x0B: //SetProtocol
                            break;
                        default:
                            errflag = 0xFF;
                            break;
                        }
                    }

                    /* �ж��Ƿ������������� */
                    if( errflag != 0xFF )
                    {
                        if( SetupReqLen > len )
                        {
                            SetupReqLen = len;
                        }
                        len = ( USBHD_Dev_SetupReqLen >= DevEP0SIZE ) ? DevEP0SIZE : USBHD_Dev_SetupReqLen;
                        memcpy( EP0_DatabufHD, pDescr, len );
                        pDescr += len;
                    }
                }
                else
                {
                    /* ������׼USB������ */
                    switch( SetupReqCode )
                    {
                        case USB_GET_DESCRIPTOR:
                        {
                            switch( ((pSetupReqPakHD->wValue)>>8) )
                            {
                                case USB_DESCR_TYP_DEVICE:
                                    /* ��ȡ�豸������ */
                                    pDescr = MyDevDescrHD;
                                    len = MyDevDescrHD[0];
                                    break;

                                case USB_DESCR_TYP_CONFIG:
                                    /* ��ȡ���������� */
                                    pDescr = MyCfgDescrHD;
                                    len = MyCfgDescrHD[2];
                                    break;

                                case USB_DESCR_TYP_STRING:
                                    /* ��ȡ�ַ��������� */
                                    switch( (pSetupReqPakHD->wValue)&0xff )
                                    {
                                        case 0:
                                            /* �����ַ��������� */
                                        pDescr = MyLangDescrHD;
                                        len = MyLangDescrHD[0];
                                            break;

                                        case 1:
                                            /* USB�����ַ��������� */
                                            pDescr = MyManuInfoHD;
                                            len = MyManuInfoHD[0];
                                            break;

                                        case 2:
                                            /* USB��Ʒ�ַ��������� */
                                            pDescr = MyProdInfoHD;
                                            len = MyProdInfoHD[0];
                                            break;

                                        case 3:
                                            /* USB���к��ַ��������� */
                                            pDescr = MySerNumInfoHD;
                                            len = sizeof( MySerNumInfoHD );
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                case USB_DESCR_TYP_REPORT:
                                    /* USB�豸���������� */
                                    pDescr = MyReportDescHD;
                                    len = sizeof( MyReportDescHD );
                                    break;

                                case USB_DESCR_TYP_QUALIF:
                                    /* �豸�޶������� */
                                    pDescr = ( PUINT8 )&MyUSBQUADescHD[ 0 ];
                                    len = sizeof( MyUSBQUADescHD );
                                    break;

                                case USB_DESCR_TYP_SPEED:
                                    /* �����ٶ����������� */
                                    /* �����ٶ����������� */
                                    if( USBHD_Dev_Speed == 0x00 )
                                    {
                                      /* ȫ��ģʽ */
                                      memcpy( &TAB_USB_FS_OSC_DESC[ 2 ], &MyCfgDescrHD[ 2 ], sizeof( MyCfgDescrHD ) - 2 );
                                      pDescr = ( PUINT8 )&TAB_USB_FS_OSC_DESC[ 0 ];
                                      len = sizeof( TAB_USB_FS_OSC_DESC );
                                    }
                                    else
                                    {
                                      errflag = 0xFF;
                                    }
                                    break;

                                case USB_DESCR_TYP_BOS:
                                    /* BOS������ */
                                    /* USB2.0�豸��֧��BOS������ */
                                    errflag = 0xFF;
                                    break;

                                default :
                                    errflag = 0xff;
                                    break;

                            }

                            if( SetupReqLen>len )   SetupReqLen = len;
                            len = (SetupReqLen >= DevEP0SIZE) ? DevEP0SIZE : SetupReqLen;
                            memcpy( pEP0_DataBuf, pDescr, len );
                            pDescr += len;
                        }
                            break;

                        case USB_SET_ADDRESS:
                            /* ���õ�ַ */
                            SetupReqLen = (pSetupReqPakHD->wValue)&0xff;
                            break;

                        case USB_GET_CONFIGURATION:
                            /* ��ȡ����ֵ */
                            pEP0_DataBuf[0] = DevConfig;
                            if ( SetupReqLen > 1 ) SetupReqLen = 1;
                            break;

                        case USB_SET_CONFIGURATION:
                            /* ��������ֵ */
                            DevConfig = (pSetupReqPakHD->wValue)&0xff;
                            break;

                        case USB_CLEAR_FEATURE:
                            /* �������� */
                            if ( ( pSetupReqPakHD->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* �����˵� */
                                switch( (pSetupReqPakHD->wIndex)&0xff )
                                {
                                case 0x82:
                                    USBOTG_FS->UEP2_TX_CTRL = (USBOTG_FS->UEP2_TX_CTRL & ~( USBHD_UEP_T_TOG|USBHD_UEP_T_RES_MASK )) | USBHD_UEP_T_RES_NAK;
                                    break;

                                case 0x02:
                                    USBOTG_FS->UEP2_RX_CTRL = (USBOTG_FS->UEP2_RX_CTRL & ~( USBHD_UEP_R_TOG|USBHD_UEP_R_RES_MASK )) | USBHD_UEP_R_RES_ACK;
                                    break;

                                case 0x81:
                                    USBOTG_FS->UEP1_TX_CTRL = (USBOTG_FS->UEP1_TX_CTRL & ~( USBHD_UEP_T_TOG|USBHD_UEP_T_RES_MASK )) | USBHD_UEP_T_RES_NAK;
                                    break;

                                case 0x01:
                                    USBOTG_FS->UEP1_RX_CTRL = (USBOTG_FS->UEP1_RX_CTRL & ~( USBHD_UEP_R_TOG|USBHD_UEP_R_RES_MASK )) | USBHD_UEP_R_RES_ACK;
                                    break;

                                default:
                                    errflag = 0xFF;
                                    break;

                                }
                            }
                            else    errflag = 0xFF;
                            break;

                        case USB_SET_FEATURE:
                            /* �������� */
                            if( ( pMySetupReqPakHD->bRequestType & 0x1F ) == 0x00 )
                            {
                                /* �����豸 */
                                if( pMySetupReqPakHD->wValue == 0x01 )
                                {
                                    if( MyCfgDescrHD[ 7 ] & 0x20 )
                                    {
                                        /* ���û���ʹ�ܱ�־ */
                                        USBHD_Dev_SleepStatus = 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( pMySetupReqPakHD->bRequestType & 0x1F ) == 0x02 )
                            {
                                /* ���ö˵� */
                                if( pMySetupReqPakHD->wValue == 0x00 )
                                {
                                    /* ����ָ���˵�STALL */
                                    switch( ( pMySetupReqPakHD->wIndex ) & 0xff )
                                    {
                                        case 0x82:
                                            /* ���ö˵�2 IN STALL */
                                            USBOTG_FS->UEP2_TX_CTRL = ( USBOTG_FS->UEP2_TX_CTRL &= ~USBHD_UEP_T_RES_MASK ) | USBHD_UEP_T_RES_STALL;
                                            //USBHS->UEP2_CTRL  = ( USBHS->UEP2_CTRL & ~USBHS_EP_T_RES_MASK ) | USBHS_EP_T_RES_STALL;
                                            break;

                                        case 0x02:
                                            /* ���ö˵�2 OUT Stall */
                                            USBOTG_FS->UEP2_RX_CTRL = ( USBOTG_FS->UEP2_RX_CTRL &= ~USBHD_UEP_R_RES_MASK ) | USBHD_UEP_R_RES_STALL;
                                            //USBHS->UEP2_CTRL  = ( USBHS->UEP2_CTRL & ~USBHS_EP_R_RES_MASK ) | USBHS_EP_R_RES_STALL;
                                            break;

                                        case 0x81:
                                            /* ���ö˵�1 IN STALL */
                                            USBOTG_FS->UEP1_TX_CTRL = ( USBOTG_FS->UEP1_TX_CTRL &= ~USBHD_UEP_T_RES_MASK ) | USBHD_UEP_T_RES_STALL;
                                            //USBHS->UEP1_CTRL  = ( USBHS->UEP1_CTRL & ~USBHS_EP_T_RES_MASK ) | USBHS_EP_T_RES_STALL;
                                            break;

                                        case 0x01:
                                            /* ���ö˵�1 OUT STALL */
                                            USBOTG_FS->UEP1_RX_CTRL = ( USBOTG_FS->UEP1_RX_CTRL &= ~USBHD_UEP_R_RES_MASK ) | USBHD_UEP_R_RES_STALL;
                                            //USBHS->UEP1_CTRL  = ( USBHS->UEP1_CTRL & ~USBHS_EP_R_RES_MASK ) | USBHS_EP_R_RES_STALL;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                        case USB_GET_INTERFACE:
                            /* ��ȡ�ӿ� */
                            pEP0_DataBuf[0] = 0x00;
                            if ( SetupReqLen > 1 ) SetupReqLen = 1;
                            break;

                        case USB_SET_INTERFACE:
                            /* ���ýӿ� */
                            EP0_DatabufHD[ 0 ] = 0x00;
                            if( USBHD_Dev_SetupReqLen > 1 )
                            {
                                USBHD_Dev_SetupReqLen = 1;
                            }
                            break;

                        case USB_GET_STATUS:
                            /* ���ݵ�ǰ�˵�ʵ��״̬����Ӧ�� */
                            EP0_DatabufHD[ 0 ] = 0x00;
                            EP0_DatabufHD[ 1 ] = 0x00;
                            if( pMySetupReqPakHD->wIndex == 0x81 )
                            {
                                if( ( USBOTG_FS->UEP1_TX_CTRL & USBHD_UEP_T_RES_MASK ) == USBHD_UEP_T_RES_STALL )
                                {
                                    EP0_DatabufHD[ 0 ] = 0x01;
                                }
                            }
                            else if( pMySetupReqPakHD->wIndex == 0x01 )
                            {
                                if( ( USBOTG_FS->UEP1_RX_CTRL & USBHD_UEP_R_RES_MASK ) == USBHD_UEP_R_RES_STALL )
                                {
                                    EP0_DatabufHD[ 0 ] = 0x01;
                                }
                            }
                            else if( pMySetupReqPakHD->wIndex == 0x82 )
                            {
                                if( ( USBOTG_FS->UEP2_TX_CTRL & USBHD_UEP_T_RES_MASK ) == USBHD_UEP_T_RES_STALL )
                                {
                                    EP0_DatabufHD[ 0 ] = 0x01;
                                }
                            }
                            else if( pMySetupReqPakHD->wIndex == 0x02 )
                            {
                                if( ( USBOTG_FS->UEP2_RX_CTRL & USBHD_UEP_R_RES_MASK ) == USBHD_UEP_R_RES_STALL )
                                {
                                    EP0_DatabufHD[ 0 ] = 0x01;
                                }
                            }
                            if( USBHD_Dev_SetupReqLen > 2 )
                            {
                                USBHD_Dev_SetupReqLen = 2;
                            }
                            break;

                        default:
                            errflag = 0xff;
                            break;
                    }
                }
                if( errflag == 0xff)
                {
#if 0
                    printf("uep0 stall\n");

#endif

                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_TOG|USBHD_UEP_T_RES_STALL;
                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_TOG|USBHD_UEP_R_RES_STALL;
                }
                else
                {
                    if( chtype & 0x80 )
                    {
                        len = (SetupReqLen>DevEP0SIZE) ? DevEP0SIZE : SetupReqLen;
                        SetupReqLen -= len;
                    }
                    else  len = 0;

                    USBOTG_FS->UEP0_TX_LEN  = len;
                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_TOG|USBHD_UEP_T_RES_ACK;
                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_TOG|USBHD_UEP_R_RES_ACK;
                }
                break;

            case USBHD_UIS_TOKEN_IN:
                switch ( USBOTG_FS->INT_ST & ( USBHD_UIS_TOKEN_MASK | USBHD_UIS_ENDP_MASK ) )
                {
                    case USBHD_UIS_TOKEN_IN:
                        switch( SetupReqCode )
                        {
                            case USB_GET_DESCRIPTOR:
                                    len = SetupReqLen >= DevEP0SIZE ? DevEP0SIZE : SetupReqLen;
                                    memcpy( pEP0_DataBuf, pDescr, len );
                                    SetupReqLen -= len;
                                    pDescr += len;
                                    USBOTG_FS->UEP0_TX_LEN   = len;
                                    USBOTG_FS->UEP0_TX_CTRL ^= USBHD_UEP_T_TOG;
                                    break;

                            case USB_SET_ADDRESS:
                                    USBOTG_FS->DEV_ADDR = (USBOTG_FS->DEV_ADDR&USBHD_UDA_GP_BIT) | SetupReqLen;
                                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
                                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
                                    break;

                            default:
                                    USBOTG_FS->UEP0_TX_LEN = 0;
                                    USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
                                    USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
                                    break;

                        }
                        break;

                case USBHD_UIS_TOKEN_IN | 1:
                    USBHD_UEP1_T_LEN=0;
                   
                    USBOTG_FS->UEP1_TX_LEN = 0;
                   // USBOTG_FS->UEP1_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP1_TX_CTRL  = (USBOTG_FS->UEP1_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    USBHD_Endp1_Up_Flag = 0x00;
                    
                    break;

                case USBHD_UIS_TOKEN_IN | 2:

                    USBOTG_FS->UEP2_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP2_TX_CTRL = (USBOTG_FS->UEP2_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    break;

                case USBHD_UIS_TOKEN_IN | 3:
                    USBOTG_FS->UEP3_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP3_TX_CTRL  = (USBOTG_FS->UEP3_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    break;

                case USBHD_UIS_TOKEN_IN | 4:
                    USBOTG_FS->UEP4_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP4_TX_CTRL  = (USBOTG_FS->UEP4_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    break;

                case USBHD_UIS_TOKEN_IN | 5:
                    USBOTG_FS->UEP5_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP5_TX_CTRL  = (USBOTG_FS->UEP5_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    break;

                case USBHD_UIS_TOKEN_IN | 6:
                    USBOTG_FS->UEP6_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP6_TX_CTRL  = (USBOTG_FS->UEP6_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    break;

                case USBHD_UIS_TOKEN_IN | 7:
                    USBOTG_FS->UEP7_TX_CTRL ^= USBHD_UEP_T_TOG;
                    USBOTG_FS->UEP7_TX_CTRL  = (USBOTG_FS->UEP7_TX_CTRL & ~USBHD_UEP_T_RES_MASK) | USBHD_UEP_T_RES_NAK;
                    break;

                default :
                    break;

                }
                break;

            case USBHD_UIS_TOKEN_OUT:
                switch ( USBOTG_FS->INT_ST & ( USBHD_UIS_TOKEN_MASK | USBHD_UIS_ENDP_MASK ) )
                {
                    case USBHD_UIS_TOKEN_OUT:
                            len = USBOTG_FS->RX_LEN;
                            break;

                    case USBHD_UIS_TOKEN_OUT | 1:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP1_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;

                            DevEP1_OUT_Deal( len );
                        }
                        break;

                    case USBHD_UIS_TOKEN_OUT | 2:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP2_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;
                            DevEP2_OUT_Deal( len );
                        }
                        break;

                    case USBHD_UIS_TOKEN_OUT | 3:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP3_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;
                            DevEP3_OUT_Deal( len );
                        }
                        break;

                    case USBHD_UIS_TOKEN_OUT | 4:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP4_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;
                            DevEP4_OUT_Deal( len );
                        }
                        break;

                    case USBHD_UIS_TOKEN_OUT | 5:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP5_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;
                            DevEP5_OUT_Deal( len );
                        }
                        break;

                    case USBHD_UIS_TOKEN_OUT | 6:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP6_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;
                            DevEP6_OUT_Deal( len );
                        }
                        break;

                    case USBHD_UIS_TOKEN_OUT | 7:
                        if ( USBOTG_FS->INT_ST & USBHD_UIS_TOG_OK )
                        {
                            USBOTG_FS->UEP7_RX_CTRL ^= USBHD_UEP_R_TOG;
                            len = USBOTG_FS->RX_LEN;
                            DevEP7_OUT_Deal( len );
                        }
                        break;
                }
#if 0
                printf( "len %d\n", len );

#endif
                break;

            case USBHD_UIS_TOKEN_SOF:

                break;

            default :
                break;

        }

        USBOTG_FS->INT_FG = USBHD_UIF_TRANSFER;
    }
    else if( intflag & USBHD_UIF_BUS_RST )
    {
        USBOTG_FS->DEV_ADDR = 0;

        USBOTG_FS->UEP0_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP1_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP2_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP3_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP4_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP5_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP6_RX_CTRL = USBHD_UEP_R_RES_ACK;
        USBOTG_FS->UEP7_RX_CTRL = USBHD_UEP_R_RES_ACK;

        USBOTG_FS->UEP0_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP1_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP2_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP3_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP4_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP5_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP6_TX_CTRL = USBHD_UEP_T_RES_NAK;
        USBOTG_FS->UEP7_TX_CTRL = USBHD_UEP_T_RES_NAK;

        USBOTG_FS->INT_FG |= USBHD_UIF_BUS_RST;
    }
    else if( intflag & USBHD_UIF_SUSPEND )
    {
        if ( USBOTG_FS->MIS_ST & USBHD_UMS_SUSPEND ) {;}
        else{;}
        USBOTG_FS->INT_FG = USBHD_UIF_SUSPEND;
    }
    else
    {
        USBOTG_FS->INT_FG = intflag;
    }
}

/*********************************************************************
 * @fn      DevEP1_IN_Deal
 *
 * @brief   Device endpoint1 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP1_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP1_TX_LEN = l;
    USBOTG_FS->UEP1_TX_CTRL = (USBOTG_FS->UEP1_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
    USBOTG_FS->UEP1_TX_CTRL ^= USBHD_UEP_T_TOG;
}

/*********************************************************************
 * @fn      DevEP2_IN_Deal
 *
 * @brief   Device endpoint2 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP2_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP2_TX_LEN = l;
    USBOTG_FS->UEP2_TX_CTRL = (USBOTG_FS->UEP2_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP3_IN_Deal
 *
 * @brief   Device endpoint3 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP3_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP3_TX_LEN = l;
    USBOTG_FS->UEP3_TX_CTRL = (USBOTG_FS->UEP3_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP4_IN_Deal
 *
 * @brief   Device endpoint4 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP4_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP4_TX_LEN = l;
    USBOTG_FS->UEP4_TX_CTRL = (USBOTG_FS->UEP4_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP5_IN_Deal
 *
 * @brief   Device endpoint5 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP5_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP5_TX_LEN = l;
    USBOTG_FS->UEP5_TX_CTRL = (USBOTG_FS->UEP5_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP6_IN_Deal
 *
 * @brief   Device endpoint6 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP6_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP6_TX_LEN = l;
    USBOTG_FS->UEP6_TX_CTRL = (USBOTG_FS->UEP6_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP7_IN_Deal
 *
 * @brief   Device endpoint7 IN.
 *
 * @param   l - IN length(<64B)
 *
 * @return  none
 */
void DevEP7_IN_Deal( UINT8 l )
{
    USBOTG_FS->UEP7_TX_LEN = l;
    USBOTG_FS->UEP7_TX_CTRL = (USBOTG_FS->UEP7_TX_CTRL & ~USBHD_UEP_T_RES_MASK)| USBHD_UEP_T_RES_ACK;
}

/*********************************************************************
 * @fn      DevEP1_OUT_Deal
 *
 * @brief   Deal device Endpoint 1 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP1_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP1_IN_DataBuf[i] = ~pEP1_OUT_DataBuf[i];
    }

    DevEP1_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP2_OUT_Deal
 *
 * @brief   Deal device Endpoint 2 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP2_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP2_IN_DataBuf[i] = ~pEP2_OUT_DataBuf[i];
    }

    DevEP2_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP3_OUT_Deal
 *
 * @brief   Deal device Endpoint 3 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP3_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP3_IN_DataBuf[i] = pEP3_OUT_DataBuf[i];
    }

    DevEP3_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP4_OUT_Deal
 *
 * @brief   Deal device Endpoint 4 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP4_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP4_IN_DataBuf[i] = pEP4_OUT_DataBuf[i];
    }

    DevEP4_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP5_OUT_Deal
 *
 * @brief   Deal device Endpoint 5 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP5_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP5_IN_DataBuf[i] = pEP5_OUT_DataBuf[i];
    }

    DevEP5_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP6_OUT_Deal
 *
 * @brief   Deal device Endpoint 6 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP6_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP6_IN_DataBuf[i] = pEP6_OUT_DataBuf[i];
    }

    DevEP6_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP7_OUT_Deal
 *
 * @brief   Deal device Endpoint 7 OUT.
 *
 * @param   l - Data length.
 *
 * @return  none
 */
void DevEP7_OUT_Deal( UINT8 l )
{
    UINT8 i;

    for(i=0; i<l; i++)
    {
        pEP7_IN_DataBuf[i] = pEP7_OUT_DataBuf[i];
    }

    DevEP7_IN_Deal( l );
}
