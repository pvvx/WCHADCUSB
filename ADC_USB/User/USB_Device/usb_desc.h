/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : header file of usb_desc.c
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef USER_USB_DESC_H_
#define USER_USB_DESC_H_

#include "debug.h"

#define USB_SET_SERIAL      1 // §å§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§ä§î §ã§Ö§â§Ú§Û§ß§í§Û §ß§à§Þ§Ö§â §Ú§Ù FLASH

#define CDC_MAX_FARME_SIZE      4096 // §Þ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§í§Û §â§Ñ§Ù§Þ§Ö§â §æ§â§Ö§Û§Þ§Ñ USB-CDC

/******************************************************************************/
/* global define */
/* file version */
#define DEF_FILE_VERSION             0x01
/* usb device info define  */
#define DEF_USB_VID                  0x1A86
#define DEF_USB_PID                  0xFE0C
/* USB device descriptor, device serial numberï¿½ï¿½bcdDeviceï¿½ï¿½ */
#define DEF_IC_PRG_VER               DEF_FILE_VERSION

/******************************************************************************/
/* usb device endpoint size define */
#define DEF_USBD_UEP0_SIZE           64     /* usb hs/fs device end-point 0 size */
/* HS */
#define DEF_USBD_HS_PACK_SIZE        512    /* usb hs device max bluk/int pack size */
#define DEF_USBD_HS_ISO_PACK_SIZE    1024   /* usb hs device max iso pack size */
/* FS */
#define DEF_USBD_FS_PACK_SIZE        64     /* usb fs device max bluk/int pack size */
#define DEF_USBD_FS_ISO_PACK_SIZE    1023   /* usb fs device max iso pack size */
/* LS */
#define DEF_USBD_LS_UEP0_SIZE        8      /* usb ls device end-point 0 size */
#define DEF_USBD_LS_PACK_SIZE        64     /* usb ls device max int pack size */

/* HS end-point size */
#define DEF_USB_EP1_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP2_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP3_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP4_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP5_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP6_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP7_HS_SIZE          DEF_USBD_HS_PACK_SIZE
/* FS end-point size */
#define DEF_USB_EP1_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP2_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP3_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP4_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP5_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP6_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP7_FS_SIZE          DEF_USBD_FS_PACK_SIZE
/* LS end-point size */
/* ... */

/******************************************************************************/
/* usb device Descriptor length, length of usb descriptors, if one descriptor not
 * exists , set the length to 0  */
//#define DEF_USBD_DEVICE_DESC_LEN     ((uint8_t)MyDevDescr[0])
//#define DEF_USBD_CONFIG_FS_DESC_LEN  ((uint16_t)MyCfgDescr_FS[2] + (uint16_t)(MyCfgDescr_FS[3] << 8))
//#define DEF_USBD_CONFIG_HS_DESC_LEN  ((uint16_t)MyCfgDescr_HS[2] + (uint16_t)(MyCfgDescr_HS[3] << 8))
#define DEF_USBD_REPORT_DESC_LEN     0
#define DEF_USBD_LANG_DESC_LEN       ((uint16_t)MyLangDescr[0])
#define DEF_USBD_MANU_DESC_LEN       ((uint16_t)MyManuInfo[0])
#define DEF_USBD_PROD_DESC_LEN       ((uint16_t)MyProdInfo[0])
#define DEF_USBD_SN_DESC_LEN         ((uint16_t)MySerNumInfo[0])
#define DEF_USBD_QUALFY_DESC_LEN     ((uint16_t)MyQuaDesc[0])
#define DEF_USBD_BOS_DESC_LEN        ((uint16_t)MyBOSDesc[2] + (uint16_t)(MyBOSDesc[3] << 8))
#define DEF_USBD_FS_OTH_DESC_LEN     (DEF_USBD_CONFIG_HS_DESC_LEN)
#define DEF_USBD_HS_OTH_DESC_LEN     (DEF_USBD_CONFIG_FS_DESC_LEN)

/******************************************************************************/
/* Configuration Descriptor */

typedef struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
} USB_IAD_DESCR;

typedef struct __attribute__((packed)) {
    uint8_t     bFunctionLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubType;
    uint16_t    bcdCDC;
} USB_CDC_HDR_DESCR;

typedef struct __attribute__ ((packed)) {
    uint8_t     bFunctionLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubType;
    uint8_t     bmCapabilities;
    uint8_t     bDataInterface;
} USB_CDC_MGMT_DESCR;

typedef struct __attribute__ ((packed)) {
    uint8_t     bFunctionLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubType;
    uint8_t     bMasterInterface0;
    uint8_t     bSlaveInterface0;
} USB_CDC_UF_DESCR;

typedef struct __attribute__ ((packed)) {
    uint8_t     bFunctionLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubType;
    uint8_t     bmCapabilities;
} USB_CDC_ACM_DESCR;

typedef struct __attribute__((packed)) _USB_CFG_DESCRS {
    USB_CFG_DESCR config;
    USB_IAD_DESCR iad0;
    USB_ITF_DESCR itf0;
    USB_CDC_HDR_DESCR cdc_hdr0;
    USB_CDC_MGMT_DESCR cdc_mgmt0;
    USB_CDC_ACM_DESCR cdc_acm0;
    USB_CDC_UF_DESCR cdc_uf0;
    USB_ENDP_DESCR ep0;
    USB_ITF_DESCR data0;
    USB_ENDP_DESCR eprx0;
    USB_ENDP_DESCR eptx0;
#if DUAL_UART
    USB_IAD_DESCR iad1;
    USB_ITF_DESCR itf1;
    USB_CDC_HDR_DESCR cdc_hdr1;
    USB_CDC_MGMT_DESCR cdc_mgmt1;
    USB_CDC_ACM_DESCR cdc_acm1;
    USB_CDC_UF_DESCR cdc_uf1;
    USB_ENDP_DESCR ep1;
    USB_ITF_DESCR data1;
    USB_ENDP_DESCR eprx1;
    USB_ENDP_DESCR eptx1;
#endif
#if 0 // USB_HID_ENABLE
    USB_ITF_DESCR hiditf;
    USB_HID_DESCR hidd;
    USB_ENDP_DESCR hideprx;
    USB_ENDP_DESCR hideptx;
#endif
} USB_CFG_DESCRS;
/******************************************************************************/
/* external variables */
extern const USB_DEV_DESCR UsbDevDescr;
extern USB_CFG_DESCRS UsbCfgDescr;
extern const uint8_t UsbLangDescr[ ];
extern const uint8_t UsbManuInfo[ ];
extern const uint8_t UsbProdInfo[ ];
#if USB_SET_SERIAL
extern uint8_t UsbSerNumInfo[ ];
#else
extern const uint8_t UsbSerNumInfo[ ];
#endif
extern const uint8_t UsbQuaDesc[ ];
extern const uint8_t UsbBOSDesc[ ];

#endif /* USER_USB_DESC_H_ */
