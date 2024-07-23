/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : usb device descriptor,configuration descriptor,
 *                      string descriptors and other descriptors.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "usb_desc.h"
#include  "ch32v30x_usbhs_device.h"

/*     ---------------------- Device Descriptor ----------------------
bLength                  : 0x12 (18 bytes)
bDescriptorType          : 0x01 (Device Descriptor)
bcdUSB                   : 0x200 (USB Version 2.00)
bDeviceClass             : 0xEF (Miscellaneous)
bDeviceSubClass          : 0x02
bDeviceProtocol          : 0x01 (IAD - Interface Association Descriptor)
bMaxPacketSize0          : 0x40 (64 bytes)
idVendor                 : 0x1A86
idProduct                : 0xFE0C
bcdDevice                : 0x0001
iManufacturer            : 0x01 (String Descriptor 1)
iProduct                 : 0x02 (String Descriptor 2)
iSerialNumber            : 0x03 (String Descriptor 3)
bNumConfigurations       : 0x01 (1 Configuration)
Data (HexDump)           : 12 01 00 02 EF 02 01 40 86 1A 0C FE 01 00 01 02 03 01 */

const USB_DEV_DESCR UsbDevDescr = {
        .bLength = sizeof(USB_DEV_DESCR),
        .bDescriptorType = USB_DESCR_TYP_DEVICE,
        .bcdUSB = th16(0x0200), //  (USB Version 2.00)
        .bDeviceClass = 0xEF, // (Miscellaneous)
        .bDeviceSubClass = 2,
        .bDeviceProtocol = 1, // (IAD - Interface Association Descriptor)
        .bMaxPacketSize0 = DEF_USBD_UEP0_SIZE,
        .idVendor = th16(DEF_USB_VID),
        .idProduct = th16(DEF_USB_PID),
        .bcdDevice = th16(DEF_IC_PRG_VER),
        .iManufacturer = DEF_STRING_DESC_MANU,
        .iProduct = DEF_STRING_DESC_PROD,
        .iSerialNumber = DEF_STRING_DESC_SERN,
        .bNumConfigurations = 1
};

USB_CFG_DESCRS UsbCfgDescr = {
    /* ------------------ Configuration Descriptor -------------------
    bLength                  : 0x09 (9 bytes)
    bDescriptorType          : 0x02 (Configuration Descriptor)
    wTotalLength             : 0x008D (141 bytes)
    bNumInterfaces           : 0x04 (4 Interfaces)
    bConfigurationValue      : 0x01 (Configuration 1)
    iConfiguration           : 0x00 (No String Descriptor)
    bmAttributes             : 0x80
     D7: Reserved, set 1     : 0x01
     D6: Self Powered        : 0x00 (no)
     D5: Remote Wakeup       : 0x00 (no)
     D4..0: Reserved, set 0  : 0x00
    MaxPower                 : 0x32 (100 mA) */
    .config = {
            .bLength = sizeof(USB_CFG_DESCR),
            .bDescriptorType = USB_DESCR_TYP_CONFIG,
            .wTotalLength = th16(sizeof(USB_CFG_DESCRS)),
            .bNumInterfaces = 2 + DUAL_UART*2 + USB_HID_ENABLE,
            .bConfigurationValue = 1, // (Configuration 1)
            .iConfiguration = 0, // (No String Descriptor)
            .bmAttributes = 0x80 | (USB_SLEEP_ENABLE << 5),
            .MaxPower = (100>>1)
    },
    /* ------------------- IAD Descriptor --------------------
    bLength                  : 0x08 (8 bytes)
    bDescriptorType          : 0x0B
    bFirstInterface          : 0x00
    bInterfaceCount          : 0x02
    bFunctionClass           : 0x02 (Communications and CDC Control)
    bFunctionSubClass        : 0x02
    bFunctionProtocol        : 0x00
    iFunction                : 0x00 (No String Descriptor)
    Data (HexDump)           : 08 0B 00 02 02 02 00 00 */
    .iad0 = {
            .bLength = sizeof(USB_IAD_DESCR),
            .bDescriptorType = 0x0B,
            .bFirstInterface = usb_interface_cdc_0,
            .bInterfaceCount = 2,
            .bFunctionClass = USB_DEV_CLASS_COMMUNIC, // Communications and CDC Control
            .bFunctionSubClass = 2,
            .bFunctionProtocol = 0,
            .iFunction = 0
    },
    /* ---------------- Interface Descriptor -----------------
    bLength                  : 0x09 (9 bytes)
    bDescriptorType          : 0x04 (Interface Descriptor)
    bInterfaceNumber         : 0x00
    bAlternateSetting        : 0x00
    bNumEndpoints            : 0x01 (1 Endpoint)
    bInterfaceClass          : 0x02 (Communications and CDC Control)
    bInterfaceSubClass       : 0x02 (Abstract Control Model)
    bInterfaceProtocol       : 0x00 (No class specific protocol required)
    iInterface               : 0x00 (No String Descriptor)
    Data (HexDump)           : 09 04 00 00 01 02 02 00 00 */
    .itf0 = {
            .bLength = sizeof(USB_ITF_DESCR),
            .bDescriptorType = USB_DESCR_TYP_INTERF,
            .bInterfaceNumber = usb_interface_cdc_0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = USB_DEV_CLASS_COMMUNIC,
            .bInterfaceSubClass = 2,
            .bInterfaceProtocol = 1, // 1?
            .iInterface = 0
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x05 (5 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x00 (Header Functional Descriptor)
    bcdCDC                   : 0x110 (CDC Version 1.10)
    Data (HexDump)           : 05 24 00 10 01 */
    .cdc_hdr0 = {
            .bFunctionLength = sizeof(USB_CDC_HDR_DESCR),
            .bDescriptorType = USB_DESCR_TYP_CS_INTF,
            .bDescriptorSubType = 0, // Header Functional Descriptor
            .bcdCDC = th16(0x0110)
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x05 (5 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x01 (Call Management Functional Descriptor)
    bmCapabilities           : 0x00
     D7..2:                  : 0x00 (Reserved)
     D1   :                  : 0x00 (sends/receives call management information only over the Communication Class interface)
     D0   :                  : 0x00 (does not handle call management itself)
    bDataInterface           : 0x01
    Data (HexDump)           : 05 24 01 00 01 */
    .cdc_mgmt0 = {
            .bFunctionLength = sizeof(USB_CDC_MGMT_DESCR),
            .bDescriptorType  = USB_DESCR_TYP_CS_INTF,
            .bDescriptorSubType = 1,
            .bmCapabilities  = 0,
            .bDataInterface = usb_interface_cdc_0 + 1
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x04 (4 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x02 (Abstract Control Management Functional Descriptor)
    bmCapabilities           : 0x02
     D7..4:                  : 0x00 (Reserved)
     D3   :                  : 0x00 (not supports the notification Network_Connection)
     D2   :                  : 0x00 (not supports the request Send_Break)
     D1   :                  : 0x01 (supports the request combination of Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State)
     D0   :                  : 0x00 (not supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and Get_Comm_Feature)
    Data (HexDump)           : 04 24 02 02 */
    .cdc_acm0 = {
        .bFunctionLength        = sizeof(USB_CDC_ACM_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_CS_INTF,
        .bDescriptorSubType     = 2,
        .bmCapabilities         = 2
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x05 (5 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x06 (Union Functional Descriptor)
    bControlInterface        : 0x00
    bSubordinateInterface[0] : 0x01
    Data (HexDump)           : 05 24 06 00 01 */
    .cdc_uf0 = {
        .bFunctionLength        = sizeof(USB_CDC_UF_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_CS_INTF,
        .bDescriptorSubType     = 6, // Union Functional Descriptor
        .bMasterInterface0      = usb_interface_cdc_0,
        .bSlaveInterface0       = usb_interface_cdc_0 + 1
    },
    /* ----------------- Endpoint Descriptor -----------------
     * Interrupt upload endpoint descriptor
    bLength                  : 0x07 (7 bytes)
    bDescriptorType          : 0x05 (Endpoint Descriptor)
    bEndpointAddress         : 0x81 (Direction=IN EndpointID=1)
    bmAttributes             : 0x03 (TransferType=Interrupt)
    wMaxPacketSize           : 0x0040 (64 bytes)
    bInterval                : 0x14 (20 ms)
    Data (HexDump)           : 07 05 81 03 40 00 14 */
    .ep0 = {
            .bLength                = sizeof(USB_ENDP_DESCR),
            .bDescriptorType        = USB_DESCR_TYP_ENDP,
            .bEndpointAddress       = DEF_UEP_IN | DEF_UEP1,
            .bmAttributes           = 3, // TransferType=Interrupt
            .wMaxPacketSizeL        = DEF_USB_EP1_FS_SIZE & 0xff,
            .wMaxPacketSizeH        = (DEF_USB_EP1_FS_SIZE >> 8) & 0xff,
            .bInterval              = 2 // 1..20 ms ?
    },
    /* ---------------- Interface Descriptor -----------------
    bLength                  : 0x09 (9 bytes)
    bDescriptorType          : 0x04 (Interface Descriptor)
    bInterfaceNumber         : 0x01
    bAlternateSetting        : 0x00
    bNumEndpoints            : 0x02 (2 Endpoints)
    bInterfaceClass          : 0x0A (CDC-Data)
    bInterfaceSubClass       : 0x00
    bInterfaceProtocol       : 0x00
    iInterface               : 0x00 (No String Descriptor)
    Data (HexDump)           : 09 04 01 00 02 0A 00 00 00 */
    .data0 = {
        .bLength                = sizeof(USB_ITF_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_INTERF,
        .bInterfaceNumber       = usb_interface_cdc_0 + 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = 0x0a, // CDC-Data
        .bInterfaceSubClass     = 0,
        .bInterfaceProtocol     = 0,
        .iInterface             = 0
    },
    /* ----------------- Endpoint Descriptor -----------------
    bLength                  : 0x07 (7 bytes)
    bDescriptorType          : 0x05 (Endpoint Descriptor)
    bEndpointAddress         : 0x02 (Direction=OUT EndpointID=2)
    bmAttributes             : 0x02 (TransferType=Bulk)
    wMaxPacketSize           : 0x0040 (64 bytes)
    bInterval                : 0x00 (ignored)
    Data (HexDump)           : 07 05 02 02 40 00 00 */
    .eprx0 = {
        .bLength                = sizeof(USB_ENDP_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_ENDP,
        .bEndpointAddress       = DEF_UEP_OUT | DEF_UEP2,
        .bmAttributes           = 2,
        .wMaxPacketSizeL        = DEF_USB_EP2_HS_SIZE & 0xff,
        .wMaxPacketSizeH        = (DEF_USB_EP2_HS_SIZE >> 8) & 0xff,
        .bInterval              = 0
    },
    /* ----------------- Endpoint Descriptor -----------------
    bLength                  : 0x07 (7 bytes)
    bDescriptorType          : 0x05 (Endpoint Descriptor)
    bEndpointAddress         : 0x83 (Direction=IN EndpointID=3)
    bmAttributes             : 0x02 (TransferType=Bulk)
    wMaxPacketSize           : 0x0040 (64 bytes)
    bInterval                : 0x00 (ignored)
    Data (HexDump)           : 07 05 83 02 40 00 00 */
    .eptx0 = {
        .bLength                = sizeof(USB_ENDP_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_ENDP,
        .bEndpointAddress       = DEF_UEP_IN | DEF_UEP2,
        .bmAttributes           = 2,
        .wMaxPacketSizeL        = DEF_USB_EP2_HS_SIZE & 0xff,
        .wMaxPacketSizeH        = (DEF_USB_EP2_HS_SIZE >> 8) & 0xff,
        .bInterval              = 0
    },
#if DUAL_UART
    /* ------------------- IAD Descriptor --------------------
    bLength                  : 0x08 (8 bytes)
    bDescriptorType          : 0x0B
    bFirstInterface          : 0x00
    bInterfaceCount          : 0x02
    bFunctionClass           : 0x02 (Communications and CDC Control)
    bFunctionSubClass        : 0x02
    bFunctionProtocol        : 0x00
    iFunction                : 0x00 (No String Descriptor)
    Data (HexDump)           : 08 0B 00 02 02 02 00 00 */
    .iad1 = {
            .bLength = sizeof(USB_IAD_DESCR),
            .bDescriptorType = 0x0B,
            .bFirstInterface = usb_interface_cdc_1,
            .bInterfaceCount = 2,
            .bFunctionClass = USB_DEV_CLASS_COMMUNIC, // Communications and CDC Control
            .bFunctionSubClass = 2,
            .bFunctionProtocol = 0,
            .iFunction = 0
    },
    /* ---------------- Interface Descriptor -----------------
    bLength                  : 0x09 (9 bytes)
    bDescriptorType          : 0x04 (Interface Descriptor)
    bInterfaceNumber         : 0x02
    bAlternateSetting        : 0x00
    bNumEndpoints            : 0x01 (1 Endpoint)
    bInterfaceClass          : 0x02 (Communications and CDC Control)
    bInterfaceSubClass       : 0x02 (Abstract Control Model)
    bInterfaceProtocol       : 0x00 (No class specific protocol required)
    iInterface               : 0x00 (No String Descriptor)
    Data (HexDump)           : 09 04 02 00 01 02 02 00 00 */
    .itf1 = {
            .bLength = sizeof(USB_ITF_DESCR),
            .bDescriptorType = USB_DESCR_TYP_INTERF,
            .bInterfaceNumber = usb_interface_cdc_1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = USB_DEV_CLASS_COMMUNIC,
            .bInterfaceSubClass = 2,
            .bInterfaceProtocol = 1, // 0
            .iInterface = 0
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x05 (5 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x00 (Header Functional Descriptor)
    bcdCDC                   : 0x110 (CDC Version 1.10)
    Data (HexDump)           : 05 24 00 10 01 */
    .cdc_hdr1 = {
            .bFunctionLength = sizeof(USB_CDC_HDR_DESCR),
            .bDescriptorType = USB_DESCR_TYP_CS_INTF,
            .bDescriptorSubType = 0, // Header Functional Descriptor
            .bcdCDC = th16(0x0110)
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x05 (5 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x01 (Call Management Functional Descriptor)
    bmCapabilities           : 0x00
     D7..2:                  : 0x00 (Reserved)
     D1   :                  : 0x00 (sends/receives call management information only over the Communication Class interface)
     D0   :                  : 0x00 (does not handle call management itself)
    bDataInterface           : 0x03
    Data (HexDump)           : 05 24 01 00 03 */
    .cdc_mgmt1 = {
            .bFunctionLength = sizeof(USB_CDC_MGMT_DESCR),
            .bDescriptorType  = USB_DESCR_TYP_CS_INTF,
            .bDescriptorSubType = 1,
            .bmCapabilities  = 0,
            .bDataInterface = usb_interface_cdc_1 + 1
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x04 (4 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x02 (Abstract Control Management Functional Descriptor)
    bmCapabilities           : 0x02
     D7..4:                  : 0x00 (Reserved)
     D3   :                  : 0x00 (not supports the notification Network_Connection)
     D2   :                  : 0x00 (not supports the request Send_Break)
     D1   :                  : 0x01 (supports the request combination of Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State)
     D0   :                  : 0x00 (not supports the request combination of Set_Comm_Feature, Clear_Comm_Feature, and Get_Comm_Feature)
    Data (HexDump)           : 04 24 02 02 */
    .cdc_acm1 = {
        .bFunctionLength        = sizeof(USB_CDC_ACM_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_CS_INTF,
        .bDescriptorSubType     = 2,
        .bmCapabilities         = 2
    },
    /* -------------- CDC Interface Descriptor ---------------
    bFunctionLength          : 0x05 (5 bytes)
    bDescriptorType          : 0x24 (Interface)
    bDescriptorSubType       : 0x06 (Union Functional Descriptor)
    bControlInterface        : 0x02
    bSubordinateInterface[0] : 0x03
    Data (HexDump)           : 05 24 06 02 03 */
    .cdc_uf1 = {
        .bFunctionLength        = sizeof(USB_CDC_UF_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_CS_INTF,
        .bDescriptorSubType     = 6, // Union Functional Descriptor
        .bMasterInterface0      = usb_interface_cdc_1,
        .bSlaveInterface0       = usb_interface_cdc_1 + 1
    },
    /* ----------------- Endpoint Descriptor -----------------
    bLength                  : 0x07 (7 bytes)
    bDescriptorType          : 0x05 (Endpoint Descriptor)
    bEndpointAddress         : 0x87 (Direction=IN EndpointID=1)
    bmAttributes             : 0x03 (TransferType=Interrupt)
    wMaxPacketSize           : 0x0040 (64 bytes)
    bInterval                : 0x14 (20 ms)
    Data (HexDump)           : 07 05 87 03 40 00 14 */
    .ep1 = {
            .bLength                = sizeof(USB_ENDP_DESCR),
            .bDescriptorType        = USB_DESCR_TYP_ENDP,
            .bEndpointAddress       = DEF_UEP_IN | DEF_UEP5,
            .bmAttributes           = 3, // TransferType=Interrupt
            .wMaxPacketSizeL        = DEF_USB_EP5_FS_SIZE & 0xff,
            .wMaxPacketSizeH        = (DEF_USB_EP5_FS_SIZE >> 8) & 0xff,
            .bInterval              = 2 // 1..20 ms
    },
    /* ---------------- Interface Descriptor -----------------
    bLength                  : 0x09 (9 bytes)
    bDescriptorType          : 0x04 (Interface Descriptor)
    bInterfaceNumber         : 0x03
    bAlternateSetting        : 0x00
    bNumEndpoints            : 0x02 (2 Endpoints)
    bInterfaceClass          : 0x0A (CDC-Data)
    bInterfaceSubClass       : 0x00
    bInterfaceProtocol       : 0x00
    iInterface               : 0x00 (No String Descriptor)
    Data (HexDump)           : 09 04 03 00 02 0A 00 00 00 */
    .data1 = {
        .bLength                = sizeof(USB_ITF_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_INTERF,
        .bInterfaceNumber       = usb_interface_cdc_1 + 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = 0x0a, // CDC-Data
        .bInterfaceSubClass     = 0,
        .bInterfaceProtocol     = 0,
        .iInterface             = 0
    },
    /* ----------------- Endpoint Descriptor -----------------
    bLength                  : 0x07 (7 bytes)
    bDescriptorType          : 0x05 (Endpoint Descriptor)
    bEndpointAddress         : 0x05 (Direction=OUT EndpointID=2)
    bmAttributes             : 0x02 (TransferType=Bulk)
    wMaxPacketSize           : 0x0040 (64 bytes)
    bInterval                : 0x00 (ignored)
    Data (HexDump)           : 07 05 05 02 40 00 00 */
    .eprx1 = {
        .bLength                = sizeof(USB_ENDP_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_ENDP,
        .bEndpointAddress       = DEF_UEP_OUT | DEF_UEP4,
        .bmAttributes           = 2,
        .wMaxPacketSizeL        = DEF_USB_EP4_HS_SIZE & 0xff,
        .wMaxPacketSizeH        = (DEF_USB_EP4_HS_SIZE >> 8) & 0xff,
        .bInterval              = 0
    },
    /* ----------------- Endpoint Descriptor -----------------
    bLength                  : 0x07 (7 bytes)
    bDescriptorType          : 0x05 (Endpoint Descriptor)
    bEndpointAddress         : 0x86 (Direction=IN EndpointID=3)
    bmAttributes             : 0x02 (TransferType=Bulk)
    wMaxPacketSize           : 0x0040 (64 bytes)
    bInterval                : 0x00 (ignored)
    Data (HexDump)           : 07 05 86 02 40 00 00 */
    .eptx1 = {
        .bLength                = sizeof(USB_ENDP_DESCR),
        .bDescriptorType        = USB_DESCR_TYP_ENDP,
        .bEndpointAddress       = DEF_UEP_IN | DEF_UEP4,
        .bmAttributes           = 2,
        .wMaxPacketSizeL        = DEF_USB_EP4_HS_SIZE & 0xff,
        .wMaxPacketSizeH        = (DEF_USB_EP4_HS_SIZE >> 8) & 0xff,
        .bInterval              = 0
    },
#endif
};

/* Language Descriptor */
const uint8_t  UsbLangDescr[ ] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
const uint8_t  UsbManuInfo[ ] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

/* Product Information */
const uint8_t  UsbProdInfo[ ] =
{
    0x16, 0x03, 'U', 0x00, 'S', 0x00, 'B', 0x00, ' ', 0x00, 'S', 0x00, 'e', 0x00,
                'r', 0x00, 'i', 0x00, 'a', 0x00, 'l', 0x00
};

/* Serial Number Information */
#if USB_SET_SERIAL
uint8_t  UsbSerNumInfo[] =
{
    0x16, 0x03, 'S', 0, 'N', 0, '0', 0, '0', 0, '0', 0, '0', 0
              , '0', 0, '0', 0, '0', 0, '0', 0
};
#else
const uint8_t  UsbSerNumInfo[ ] =
{
    0x16, 0x03, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, '5', 0x00
              , '6', 0x00, '7', 0x00, '8', 0x00, '9', 0x00
};
#endif

/* Device Qualified Descriptor */
const uint8_t UsbQuaDesc[ ] =
{
    0x0A, 0x06, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x40, 0x01, 0x00,
};

/* Device BOS Descriptor */
const uint8_t UsbBOSDesc[ ] =
{
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};

#if 0
/* USB Full-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_FS_OSC_DESC[ sizeof(UsbCfgDescr) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};

/* USB High-Speed Mode, Other speed configuration Descriptor */
uint8_t TAB_USB_HS_OSC_DESC[ sizeof(UsbCfgDescr_FS) ] =
{
    /* Other parts are copied through the program */
    0x09, 0x07,
};
#endif
