
#ifndef _MV_CAMERA_PARAMS_H_
#define _MV_CAMERA_PARAMS_H_

#include "PixelType.h"

#ifndef __cplusplus
typedef char    bool;
#define true    1
#define false   0
#endif

/// \~chinese ����ʽ                  \~english The Method of Sorting
typedef enum _MV_SORT_METHOD_
{
    SortMethod_SerialNumber   = 0,                  ///< \~chinese �����к�����                     \~english Sorting by SerialNumber
    SortMethod_UserID         = 1,                  ///< \~chinese ���û��Զ�����������             \~english Sorting by UserID
    SortMethod_CurrentIP_ASC  = 2,                  ///< \~chinese ����ǰIP��ַ��������ֻ��GEV�����Ч���������������Ĭ������         \~english Sorting by current IP��Ascending, Available for GEV cameras only. Other types of cameras are sorted by default��
    SortMethod_CurrentIP_DESC = 3,                  ///< \~chinese ����ǰIP��ַ���򣨽���ֻ��GEV�����Ч���������������Ĭ������         \~english Sorting by current IP��Descending, Available for GEV cameras only. Other types of cameras are sorted by default��

}MV_SORT_METHOD;


/// \~chinese GigE�豸��Ϣ              \~english GigE device info
typedef struct _MV_GIGE_DEVICE_INFO_
{
    unsigned int        nIpCfgOption;                               ///< [OUT] \~chinese IP����ѡ��             \~english IP Configuration Options
    unsigned int        nIpCfgCurrent;                              ///< [OUT] \~chinese ��ǰIP����             \~english IP Configuration
    unsigned int        nCurrentIp;                                 ///< [OUT] \~chinese ��ǰIP��ַ             \~english Current Ip
    unsigned int        nCurrentSubNetMask;                         ///< [OUT] \~chinese ��ǰ��������           \~english Curtent Subnet Mask
    unsigned int        nDefultGateWay;                             ///< [OUT] \~chinese ��ǰ����               \~english Current Gateway
    unsigned char       chManufacturerName[32];                     ///< [OUT] \~chinese ����������             \~english Manufacturer Name
    unsigned char       chModelName[32];                            ///< [OUT] \~chinese �ͺ�����               \~english Model Name
    unsigned char       chDeviceVersion[32];                        ///< [OUT] \~chinese �豸�汾               \~english Device Version 
    unsigned char       chManufacturerSpecificInfo[48];             ///< [OUT] \~chinese �����̵ľ�����Ϣ       \~english Manufacturer Specific Information
    unsigned char       chSerialNumber[16];                         ///< [OUT] \~chinese ���к�                 \~english Serial Number
    unsigned char       chUserDefinedName[16];                      ///< [OUT] \~chinese �û��Զ�������         \~english User Defined Name 
    unsigned int        nNetExport;                                 ///< [OUT] \~chinese ����IP��ַ             \~english NetWork IP Address

    unsigned int        nReserved[4];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_GIGE_DEVICE_INFO;

///< \~chinese ����������Ϣ��С       \~english Maximum data information size
#define INFO_MAX_BUFFER_SIZE            64

/// \~chinese USB�豸��Ϣ               \~english USB device info
typedef struct _MV_USB3_DEVICE_INFO_
{
    unsigned char       CrtlInEndPoint;                             ///< [OUT] \~chinese ��������˵�           \~english Control input endpoint
    unsigned char       CrtlOutEndPoint;                            ///< [OUT] \~chinese ��������˵�           \~english Control output endpoint
    unsigned char       StreamEndPoint;                             ///< [OUT] \~chinese ���˵�                 \~english Flow endpoint
    unsigned char       EventEndPoint;                              ///< [OUT] \~chinese �¼��˵�               \~english Event endpoint
    unsigned short      idVendor;                                   ///< [OUT] \~chinese ��Ӧ��ID��             \~english Vendor ID Number
    unsigned short      idProduct;                                  ///< [OUT] \~chinese ��ƷID��               \~english Device ID Number
    unsigned int        nDeviceNumber;                              ///< [OUT] \~chinese �豸������             \~english Device Number
    unsigned char       chDeviceGUID[INFO_MAX_BUFFER_SIZE];         ///< [OUT] \~chinese �豸GUID��             \~english Device GUID Number
    unsigned char       chVendorName[INFO_MAX_BUFFER_SIZE];         ///< [OUT] \~chinese ��Ӧ������             \~english Vendor Name
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];          ///< [OUT] \~chinese �ͺ�����               \~english Model Name
    unsigned char       chFamilyName[INFO_MAX_BUFFER_SIZE];         ///< [OUT] \~chinese ��������               \~english Family Name
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];      ///< [OUT] \~chinese �豸�汾               \~english Device Version
    unsigned char       chManufacturerName[INFO_MAX_BUFFER_SIZE];   ///< [OUT] \~chinese ����������             \~english Manufacturer Name
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];       ///< [OUT] \~chinese ���к�                 \~english Serial Number
    unsigned char       chUserDefinedName[INFO_MAX_BUFFER_SIZE];    ///< [OUT] \~chinese �û��Զ�������         \~english User Defined Name
    unsigned int        nbcdUSB;                                    ///< [OUT] \~chinese ֧�ֵ�USBЭ��          \~english Support USB Protocol
    unsigned int        nDeviceAddress;                             ///< [OUT] \~chinese �豸��ַ               \~english Device Address
	unsigned int        nReserved[2];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_USB3_DEVICE_INFO;

/// \~chinese CameraLink�豸��Ϣ        \~english CameraLink device info
typedef struct _MV_CamL_DEV_INFO_
{
    unsigned char       chPortID[INFO_MAX_BUFFER_SIZE];             ///< [OUT] \~chinese ���ں�                 \~english Port ID
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];          ///< [OUT] \~chinese �ͺ�����               \~english Model Name
    unsigned char       chFamilyName[INFO_MAX_BUFFER_SIZE];         ///< [OUT] \~chinese ����                   \~english Family Name
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];      ///< [OUT] \~chinese �豸�汾               \~english Device Version
    unsigned char       chManufacturerName[INFO_MAX_BUFFER_SIZE];   ///< [OUT] \~chinese ����������             \~english Manufacturer Name
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];       ///< [OUT] \~chinese ���к�                 \~english Serial Number

    unsigned int        nReserved[38];                              ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CamL_DEV_INFO;



///< \~chinese CoaXPress�����Ϣ      \~english CoaXPress device information
typedef struct _MV_CXP_DEVICE_INFO_
{
    unsigned char       chInterfaceID[INFO_MAX_BUFFER_SIZE];     ///  \~chinese �ɼ���ID       \~english Interface ID of Frame Grabber
    unsigned char       chVendorName[INFO_MAX_BUFFER_SIZE];      ///< \~chinese ��Ӧ������       \~english Vendor name
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];       ///< \~chinese �ͺ�����         \~english Model name
    unsigned char       chManufacturerInfo[INFO_MAX_BUFFER_SIZE];///< \~chinese ������Ϣ         \~english Manufacturer information
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];   ///< \~chinese ����汾         \~english Device version
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];    ///< \~chinese ���к�           \~english Serial number
    unsigned char       chUserDefinedName[INFO_MAX_BUFFER_SIZE]; ///< \~chinese �û��Զ�������   \~english User defined name
    unsigned char       chDeviceID[INFO_MAX_BUFFER_SIZE];        ///< \~chinese ���ID            \~english Device ID
    unsigned int        nReserved[7];                              ///< \~chinese �����ֶ�      \~english Reserved
}MV_CXP_DEVICE_INFO;

///< \~chinese �ɼ���Camera Link�����Ϣ          \~english Camera Link device information on frame grabber
typedef struct _MV_CML_DEVICE_INFO_
{
    unsigned char       chInterfaceID[INFO_MAX_BUFFER_SIZE];     ///  \~chinese �ɼ���ID       \~english Interface ID of Frame Grabber
    unsigned char       chVendorName[INFO_MAX_BUFFER_SIZE];      ///< \~chinese ��Ӧ������       \~english Vendor name
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];       ///< \~chinese �ͺ�����         \~english Model name
    unsigned char       chManufacturerInfo[INFO_MAX_BUFFER_SIZE];///< \~chinese ������Ϣ         \~english Manufacturer information
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];   ///< \~chinese ����汾         \~english Device version
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];    ///< \~chinese ���к�           \~english Serial number
    unsigned char       chUserDefinedName[INFO_MAX_BUFFER_SIZE]; ///< \~chinese �û��Զ�������   \~english User defined name
    unsigned char       chDeviceID[INFO_MAX_BUFFER_SIZE];        ///< \~chinese ���ID            \~english Device ID
    unsigned int        nReserved[7];                              ///< \~chinese �����ֶ�      \~english Reserved
}MV_CML_DEVICE_INFO;

///< \~chinese XoFLink�����Ϣ      \~english XoFLink device information
typedef struct _MV_XOF_DEVICE_INFO_
{
    unsigned char       chInterfaceID[INFO_MAX_BUFFER_SIZE];     ///  \~chinese �ɼ���ID       \~english Interface ID of Frame Grabber
    unsigned char       chVendorName[INFO_MAX_BUFFER_SIZE];      ///< \~chinese ��Ӧ������       \~english Vendor name
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];       ///< \~chinese �ͺ�����         \~english Model name
    unsigned char       chManufacturerInfo[INFO_MAX_BUFFER_SIZE];///< \~chinese ������Ϣ         \~english Manufacturer information
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];   ///< \~chinese ����汾         \~english Device version
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];    ///< \~chinese ���к�           \~english Serial number
    unsigned char       chUserDefinedName[INFO_MAX_BUFFER_SIZE]; ///< \~chinese �û��Զ�������   \~english User defined name
    unsigned char       chDeviceID[INFO_MAX_BUFFER_SIZE];        ///< \~chinese ���ID            \~english Device ID
    unsigned int        nReserved[7];                              ///< \~chinese �����ֶ�      \~english Reserved
}MV_XOF_DEVICE_INFO;

///< \~chinese �豸�����Э������       \~english Device Transport Layer Protocol Type
#define MV_UNKNOW_DEVICE                0x00000000                  ///< \~chinese δ֪�豸���ͣ���������       \~english Unknown Device Type, Reserved 
#define MV_GIGE_DEVICE                  0x00000001                  ///< \~chinese GigE�豸                     \~english GigE Device
#define MV_1394_DEVICE                  0x00000002                  ///< \~chinese 1394-a/b �豸                \~english 1394-a/b Device
#define MV_USB_DEVICE                   0x00000004                  ///< \~chinese USB �豸                     \~english USB Device
#define MV_CAMERALINK_DEVICE            0x00000008                  ///< \~chinese CameraLink�豸               \~english CameraLink Device
#define MV_VIR_GIGE_DEVICE              0x00000010                  ///< \~chinese ����GigE�豸                 \~english Virtual GigE Device
#define MV_VIR_USB_DEVICE               0x00000020                  ///< \~chinese ����USB�豸                 \~english Virtual USB Device
#define MV_GENTL_GIGE_DEVICE            0x00000040                  ///< \~chinese ����������GigE�豸          \~english GenTL GigE Device
#define MV_GENTL_CAMERALINK_DEVICE      0x00000080                  ///< \~chinese CameraLink����豸          \~english GenTL CameraLink Camera Device   
#define MV_GENTL_CXP_DEVICE             0x00000100                  ///< \~chinese CoaXPress�豸              \~english GenTL CoaXPress Device
#define MV_GENTL_XOF_DEVICE             0x00000200                  ///< \~chinese XoF�豸                    \~english GenTL XoF Device

/// \~chinese �豸��Ϣ                  \~english Device info
typedef struct _MV_CC_DEVICE_INFO_
{
    unsigned short          nMajorVer;                              ///< [OUT] \~chinese ��Ҫ�汾               \~english Major Version
    unsigned short          nMinorVer;                              ///< [OUT] \~chinese ��Ҫ�汾               \~english Minor Version
    unsigned int            nMacAddrHigh;                           ///< [OUT] \~chinese ��MAC��ַ              \~english High MAC Address
    unsigned int            nMacAddrLow;                            ///< [OUT] \~chinese ��MAC��ַ              \~english Low MAC Address
    unsigned int            nTLayerType;                            ///< [OUT] \~chinese �豸�����Э������     \~english Device Transport Layer Protocol Type

    unsigned int            nDevTypeInfo;                           ///< [OUT] \~chinese �豸������Ϣ         \~english Device Type Info     
                                                                    ///<  \~chinese �豸���͹���         \~english Device Type Rules
                                                                    ///< 7 - 0 bit:      \~chinese Ԥ��           \~english  Reserved
                                                                    ///< 15 - 8 bit��    \~chinese ��Ʒ�����     \~english  Product subtype
                                                                    ///< 23 - 16 bit��   \~chinese ��Ʒ����       \~english  product type
                                                                    ///< 31 - 24bit��    \~chinese ��Ʒ��         \~english  Product Line  ��Ʒ��   //eg: 0x01 ��׼��Ʒ/2D Produces;  0x02 3D��Ʒ/3D Produces ; 0x03 ����ID��Ʒ/Intelligent ID products    
    unsigned int            nReserved[3];                           ///<       \~chinese Ԥ��                   \~english Reserved

    union
    {
        MV_GIGE_DEVICE_INFO stGigEInfo;                             ///< [OUT] \~chinese GigE�豸��Ϣ              \~english GigE Device Info
        MV_USB3_DEVICE_INFO stUsb3VInfo;                            ///< [OUT] \~chinese USB�豸��Ϣ               \~english USB Device Info
        MV_CamL_DEV_INFO    stCamLInfo;                             ///< [OUT] \~chinese CameraLink�豸��Ϣ        \~english CameraLink Device Info
        MV_CML_DEVICE_INFO  stCMLInfo;                              ///< [OUT] \~chinese �ɼ���CameraLink�豸��Ϣ     \~english CameraLink Device Info On Frame Grabber
        MV_CXP_DEVICE_INFO  stCXPInfo;                              ///< [OUT] \~chinese �ɼ���CoaXPress�豸��Ϣ     \~english CoaXPress Device Info On Frame Grabber
        MV_XOF_DEVICE_INFO  stXoFInfo;                              ///< [OUT] \~chinese �ɼ���XoF�豸��Ϣ          \~english XoF Device Info On Frame Grabber
    }SpecialInfo;

}MV_CC_DEVICE_INFO;

///< \~chinese ���֧�ֵĴ����ʵ������ \~english The maximum number of supported transport layer instances
#define MV_MAX_TLS_NUM                  8
///< \~chinese ���֧�ֵ��豸����       \~english The maximum number of supported devices
#define MV_MAX_DEVICE_NUM               256

/// \~chinese �豸��Ϣ�б�              \~english Device Information List
typedef struct _MV_CC_DEVICE_INFO_LIST_
{
    unsigned int        nDeviceNum;                                 ///< [OUT] \~chinese �����豸����           \~english Online Device Number
    MV_CC_DEVICE_INFO*  pDeviceInfo[MV_MAX_DEVICE_NUM];             ///< [OUT] \~chinese ֧�����256���豸      \~english Support up to 256 devices

}MV_CC_DEVICE_INFO_LIST;


///< \~chinese �ɼ�������         \~english Interface type
#define MV_GIGE_INTERFACE                    0x00000001          ///< \~chinese GigE Vision�ɼ���        \~english GigE Vision interface
#define MV_CAMERALINK_INTERFACE              0x00000004          ///< \~chinese Camera Link�ɼ���        \~english Camera Link interface
#define MV_CXP_INTERFACE                     0x00000008          ///< \~chinese CoaXPress�ɼ���          \~english CoaXPress interface
#define MV_XOF_INTERFACE                     0x00000010          ///< \~chinese XoFLink�ɼ���            \~english XoFLink interface

///< \~chinese ���֧�ֵĲɼ�������  \~english The maximum number of Frame Grabber interface supported
#define MV_MAX_INTERFACE_NUM             64

///< \~chinese �ɼ�����Ϣ            \~english Interface information
typedef struct _MV_INTERFACE_INFO_
{
    unsigned int        nTLayerType;                                ///< \~chinese �ɼ�������    \~english Interface type
    // ��16λ��Ч: bits(0~2)������, bits(3~7)�������, bits(8-15)��������
    // The lower 16 bits are valid: bits (0~2) represents the function, bits (3~7) represents the device, and bits (8~15) represents the bus
    // |15  14  13  12  11  10  9  8 | 7  6  5  4  3 | 2  1  0 |
    // +-----------------------------+---------------+---------+
    // |             bus             |    device     |  func   |
    unsigned int        nPCIEInfo;                                ///< \~chinese �ɼ�����PCIE�����Ϣ  \~english PCIe slot information of interface
    unsigned char       chInterfaceID[INFO_MAX_BUFFER_SIZE];      ///< \~chinese �ɼ���ID    \~english Interface ID
    unsigned char       chDisplayName[INFO_MAX_BUFFER_SIZE];      ///< \~chinese ��ʾ����    \~english Display name
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];     ///< \~chinese ���к�      \~english Serial number
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];        ///< [OUT] \~chinese �ͺ�       \~english model name
    unsigned char       chManufacturer[INFO_MAX_BUFFER_SIZE];     ///< [OUT] \~chinese ����       \~english manufacturer name
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];    ///< [OUT] \~chinese �汾��     \~english device version
    unsigned char       chUserDefinedName[INFO_MAX_BUFFER_SIZE];  ///< [OUT] \~chinese �Զ������� \~english user defined name
    unsigned int        nReserved[64];                            ///< \~chinese �����ֶ�     \~english Reserved
}MV_INTERFACE_INFO;

///< \~chinese �ɼ�����Ϣ�б�           \~english Interface Information List
typedef struct _MV_INTERFACE_INFO_LIST_
{
    unsigned int nInterfaceNum;                                   ///< [OUT] \~chinese �ɼ�������                      \~english Interface Number
    MV_INTERFACE_INFO* pInterfaceInfos[MV_MAX_INTERFACE_NUM];     ///< [OUT] \~chinese �ɼ�����Ϣ, ֧�����64���豸     \~english Information of interfaces, support up to 64 interfaces
}MV_INTERFACE_INFO_LIST;



/// \~chinese ͨ��GenTLö�ٵ��Ľӿ���Ϣ \~english Interface Information with GenTL
typedef struct _MV_GENTL_IF_INFO_
{
    unsigned char       chInterfaceID[INFO_MAX_BUFFER_SIZE];        ///< [OUT] \~chinese GenTL�ӿ�ID            \~english Interface ID
    unsigned char       chTLType[INFO_MAX_BUFFER_SIZE];             ///< [OUT] \~chinese ���������             \~english GenTL Type
    unsigned char       chDisplayName[INFO_MAX_BUFFER_SIZE];        ///< [OUT] \~chinese Interface��ʾ����      \~english Display Name
    unsigned int        nCtiIndex;                                  ///< [OUT] \~chinese GenTL��cti�ļ�����     \~english The Index of Cti Files

    unsigned int        nReserved[8];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_GENTL_IF_INFO;

///< \~chinese ���֧�ֵ�GenTL�ӿ�����  \~english The maximum number of GenTL interface supported
#define MV_MAX_GENTL_IF_NUM             256

/// \~chinese ͨ��GenTLö�ٵ��Ľӿ���Ϣ�б� \~english Inferface Information List with GenTL
typedef struct _MV_GENTL_IF_INFO_LIST_
{
    unsigned int        nInterfaceNum;                              ///< [OUT] \~chinese ���߽ӿ�����           \~english Online Inferface Number
    MV_GENTL_IF_INFO*   pIFInfo[MV_MAX_GENTL_IF_NUM];               ///< [OUT] \~chinese ֧�����256���ӿ�      \~english Support up to 256 inferfaces

}MV_GENTL_IF_INFO_LIST;

/// \~chinese ͨ��GenTLö�ٵ����豸��Ϣ \~english Device Information with GenTL
typedef struct _MV_GENTL_DEV_INFO_
{
    unsigned char       chInterfaceID[INFO_MAX_BUFFER_SIZE];        ///< [OUT] \~chinese GenTL�ӿ�ID            \~english Interface ID
    unsigned char       chDeviceID[INFO_MAX_BUFFER_SIZE];           ///< [OUT] \~chinese �豸ID                 \~english Device ID
    unsigned char       chVendorName[INFO_MAX_BUFFER_SIZE];         ///< [OUT] \~chinese ��Ӧ������             \~english Vendor Name
    unsigned char       chModelName[INFO_MAX_BUFFER_SIZE];          ///< [OUT] \~chinese �ͺ�����               \~english Model Name
    unsigned char       chTLType[INFO_MAX_BUFFER_SIZE];             ///< [OUT] \~chinese ���������             \~english GenTL Type
    unsigned char       chDisplayName[INFO_MAX_BUFFER_SIZE];        ///< [OUT] \~chinese �豸��ʾ����           \~english Display Name
    unsigned char       chUserDefinedName[INFO_MAX_BUFFER_SIZE];    ///< [OUT] \~chinese �û��Զ�������         \~english User Defined Name
    unsigned char       chSerialNumber[INFO_MAX_BUFFER_SIZE];       ///< [OUT] \~chinese ���к�                 \~english Serial Number
    unsigned char       chDeviceVersion[INFO_MAX_BUFFER_SIZE];      ///< [OUT] \~chinese �豸�汾��             \~english Device Version
    unsigned int        nCtiIndex;                                  ///< [OUT] \~chinese GenTL��cti�ļ�����     \~english The Index of Cti Files

    unsigned int        nReserved[8];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_GENTL_DEV_INFO;

///< \~chinese ���֧�ֵ�GenTL�豸����  \~english The maximum number of GenTL devices supported
#define MV_MAX_GENTL_DEV_NUM            256

/// \~chinese ͨ��GenTLö�ٵ����豸��Ϣ�б� \~english Device Information List with GenTL
typedef struct _MV_GENTL_DEV_INFO_LIST_
{
    unsigned int        nDeviceNum;                                 ///< [OUT] \~chinese �����豸����           \~english Online Device Number
    MV_GENTL_DEV_INFO*  pDeviceInfo[MV_MAX_GENTL_DEV_NUM];          ///< [OUT] \~chinese ֧�����256���豸      \~english Support up to 256 devices

}MV_GENTL_DEV_INFO_LIST;

/// \~chinese �豸�ķ���ģʽ            \~english Device Access Mode
#define MV_ACCESS_Exclusive                     1                   /// \~chinese ��ռȨ�ޣ�����APPֻ�����CCP�Ĵ���                    \~english Exclusive authority, other APP is only allowed to read the CCP register
#define MV_ACCESS_ExclusiveWithSwitch           2                   /// \~chinese ���Դ�5ģʽ����ռȨ�ޣ�Ȼ���Զ�ռȨ�޴�             \~english You can seize the authority from the 5 mode, and then open with exclusive authority
#define MV_ACCESS_Control                       3                   /// \~chinese ����Ȩ�ޣ�����APP��������мĴ���                     \~english Control authority, allows other APP reading all registers
#define MV_ACCESS_ControlWithSwitch             4                   /// \~chinese ���Դ�5��ģʽ����ռȨ�ޣ�Ȼ���Կ���Ȩ�޴�           \~english You can seize the authority from the 5 mode, and then open with control authority
#define MV_ACCESS_ControlSwitchEnable           5                   /// \~chinese �Կɱ���ռ�Ŀ���Ȩ�޴�                              \~english Open with seized control authority
#define MV_ACCESS_ControlSwitchEnableWithKey    6                   /// \~chinese ���Դ�5��ģʽ����ռȨ�ޣ�Ȼ���Կɱ���ռ�Ŀ���Ȩ�޴� \~english You can seize the authority from the 5 mode, and then open with seized control authority
#define MV_ACCESS_Monitor                       7                   /// \~chinese ��ģʽ���豸�������ڿ���Ȩ����                      \~english Open with read mode and is available under control authority

/// \~chinese Chunk����                 \~english The content of ChunkData
typedef struct _MV_CHUNK_DATA_CONTENT_
{
    unsigned char*      pChunkData;                                 ///< [OUT] \~chinese Chunk����              \~english Chunk Data
    unsigned int        nChunkID;                                   ///< [OUT] \~chinese Chunk ID               \~english Chunk ID
    unsigned int        nChunkLen;                                  ///< [OUT] \~chinese Chunk�ĳ���            \~english Chunk Length

    unsigned int        nReserved[8];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CHUNK_DATA_CONTENT;


/// \~chinese ���֡����Ϣ              \~english Output Frame Information
typedef struct _MV_FRAME_OUT_INFO_EX_
{
    unsigned short          nWidth;                                 ///< [OUT] \~chinese ͼ���(���65535����������nExtendWidth)    \~english Image Width (over 65535, use nExtendWidth)
    unsigned short          nHeight;                                ///< [OUT] \~chinese ͼ���(���65535����������nExtendHeight)   \~english Image Height(over 65535, use nExtendHeight)
    enum MvGvspPixelType    enPixelType;                            ///< [OUT] \~chinese ���ظ�ʽ               \~english Pixel Type

    unsigned int            nFrameNum;                              ///< [OUT] \~chinese ֡��                   \~english Frame Number
    unsigned int            nDevTimeStampHigh;                      ///< [OUT] \~chinese ʱ�����32λ           \~english Timestamp high 32 bits
    unsigned int            nDevTimeStampLow;                       ///< [OUT] \~chinese ʱ�����32λ           \~english Timestamp low 32 bits
    unsigned int            nReserved0;                             ///< [OUT] \~chinese ������8�ֽڶ���        \~english Reserved, 8-byte aligned
    int64_t                 nHostTimeStamp;                         ///< [OUT] \~chinese �������ɵ�ʱ���       \~english Host-generated timestamp

    unsigned int            nFrameLen;                              ///< [OUT] \~chinese ֡�ĳ���               \~english The Length of Frame

    /// \~chinese �豸ˮӡʱ��      \~english Device frame-specific time scale
    unsigned int            nSecondCount;                           ///< [OUT] \~chinese ����                   \~english The Seconds
    unsigned int            nCycleCount;                            ///< [OUT] \~chinese ������                 \~english The Count of Cycle
    unsigned int            nCycleOffset;                           ///< [OUT] \~chinese ����ƫ����             \~english The Offset of Cycle

    float                   fGain;                                  ///< [OUT] \~chinese ����                   \~english Gain
    float                   fExposureTime;                          ///< [OUT] \~chinese �ع�ʱ��               \~english Exposure Time
    unsigned int            nAverageBrightness;                     ///< [OUT] \~chinese ƽ������               \~english Average brightness

    /// \~chinese ��ƽ�����        \~english White balance
    unsigned int            nRed;                                   ///< [OUT] \~chinese ��ɫ                   \~english Red
    unsigned int            nGreen;                                 ///< [OUT] \~chinese ��ɫ                   \~english Green
    unsigned int            nBlue;                                  ///< [OUT] \~chinese ��ɫ                   \~english Blue

    unsigned int            nFrameCounter;                          ///< [OUT] \~chinese ��֡��                 \~english Frame Counter
    unsigned int            nTriggerIndex;                          ///< [OUT] \~chinese ��������               \~english Trigger Counting

    unsigned int            nInput;                                 ///< [OUT] \~chinese ����                   \~english Input
    unsigned int            nOutput;                                ///< [OUT] \~chinese ���                   \~english Output

    /// \~chinese ROI����           \~english ROI Region
    unsigned short          nOffsetX;                               ///< [OUT] \~chinese ˮƽƫ����             \~english OffsetX
    unsigned short          nOffsetY;                               ///< [OUT] \~chinese ��ֱƫ����             \~english OffsetY
    unsigned short          nChunkWidth;                            ///< [OUT] \~chinese Chunk��                \~english The Width of Chunk
    unsigned short          nChunkHeight;                           ///< [OUT] \~chinese Chunk��                \~english The Height of Chunk

    unsigned int            nLostPacket;                            ///< [OUT] \~chinese ��֡������             \~english Lost Packet Number In This Frame

    unsigned int            nUnparsedChunkNum;                      ///< [OUT] \~chinese δ������Chunkdata����  \~english Unparsed Chunk Number
    union
    {
        MV_CHUNK_DATA_CONTENT*  pUnparsedChunkContent;              ///< [OUT] \~chinese δ������Chunk          \~english Unparsed Chunk Content
        int64_t                 nAligning;                          ///< [OUT] \~chinese У׼                   \~english Aligning
    }UnparsedChunkList;

    unsigned int            nExtendWidth;                           ///< [OUT] \~chinese ͼ���(��չ����)       \~english Image Width
    unsigned int            nExtendHeight;                          ///< [OUT] \~chinese ͼ���(��չ����)       \~english Image Height

    unsigned int            nReserved[34];                          ///<       \~chinese Ԥ��                   \~english Reserved

}MV_FRAME_OUT_INFO_EX;

/// \~chinese ͼ��ṹ�壬���ͼ���ַ��ͼ����Ϣ    \~english Image Struct, output the pointer of Image and the information of the specific image
typedef struct _MV_FRAME_OUT_
{
    unsigned char*          pBufAddr;                               ///< [OUT] \~chinese ͼ��ָ���ַ           \~english  pointer of image
    MV_FRAME_OUT_INFO_EX    stFrameInfo;                            ///< [OUT] \~chinese ͼ����Ϣ               \~english information of the specific image

    unsigned int            nRes[16];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_FRAME_OUT;

/// \~chinese ȡ������                  \~english The strategy of Grabbing
typedef enum _MV_GRAB_STRATEGY_
{
    MV_GrabStrategy_OneByOne            = 0,                        ///< \~chinese �Ӿɵ���һ֡һ֡�Ļ�ȡͼ��   \~english Grab One By One
    MV_GrabStrategy_LatestImagesOnly    = 1,                        ///< \~chinese ��ȡ�б������µ�һ֡ͼ��     \~english Grab The Latest Image
    MV_GrabStrategy_LatestImages        = 2,                        ///< \~chinese ��ȡ�б������µ�ͼ��         \~english Grab The Latest Images
    MV_GrabStrategy_UpcomingImage       = 3,                        ///< \~chinese �ȴ���һ֡ͼ��               \~english Grab The Upcoming Image

}MV_GRAB_STRATEGY;

/// \~chinese ���紫��������Ϣ        \~english Network transmission information
typedef struct _MV_NETTRANS_INFO_
{
    int64_t             nReceiveDataSize;                           ///< [OUT] \~chinese �ѽ������ݴ�С[Start��Stop֮��]    \~english Received Data Size
    int                 nThrowFrameCount;                           ///< [OUT] \~chinese ��֡����                           \~english Throw frame number
    unsigned int        nNetRecvFrameCount;                         ///< [OUT] \~chinese �ѽ��յ�֡��                       \~english Received Frame Count
    int64_t             nRequestResendPacketCount;                  ///< [OUT] \~chinese �����ط�����                       \~english Request Resend Packet Count
    int64_t             nResendPacketCount;                         ///< [OUT] \~chinese �ط�����                           \~english Resend Packet Count

}MV_NETTRANS_INFO;

/// \~chinese ��Ϣ����                  \~english Information Type
#define MV_MATCH_TYPE_NET_DETECT        0x00000001                  ///< \~chinese ���������Ͷ�����Ϣ               \~english Network traffic and packet loss information
#define MV_MATCH_TYPE_USB_DETECT        0x00000002                  ///< \~chinese host���յ�����U3V�豸���ֽ�����  \~english The total number of bytes host received from U3V device

/// \~chinese ȫƥ���һ����Ϣ�ṹ��    \~english A fully matched information structure
typedef struct _MV_ALL_MATCH_INFO_
{
    unsigned int        nType;                                      ///< [IN]  \~chinese ��Ҫ�������Ϣ���ͣ�e.g. MV_MATCH_TYPE_NET_DETECT��MV_MATCH_TYPE_USB_DETECT  \~english Information type need to output ,e.g. MV_MATCH_TYPE_NET_DETECT��MV_MATCH_TYPE_USB_DETECT
    void*               pInfo;                                      ///< [OUT] \~chinese �������Ϣ���棬�ɵ����߷���                       \~english Output information cache, which is allocated by the caller
    unsigned int        nInfoSize;                                  ///< [IN]  \~chinese ��Ϣ����Ĵ�С                                     \~english Information cache size

}MV_ALL_MATCH_INFO;

/// \~chinese ���������Ͷ�����Ϣ�����ṹ�壬��Ӧ����Ϊ MV_MATCH_TYPE_NET_DETECT     \~english Network traffic and packet loss feedback structure, the corresponding type is MV_MATCH_TYPE_NET_DETECT
typedef struct _MV_MATCH_INFO_NET_DETECT_
{
    int64_t             nReceiveDataSize;                           ///< [OUT] \~chinese �ѽ������ݴ�С[Start��Stop֮��]       \~english Received data size 
    int64_t             nLostPacketCount;                           ///< [OUT] \~chinese ��ʧ�İ�����                          \~english Number of packets lost
    unsigned int        nLostFrameCount;                            ///< [OUT] \~chinese ��֡����                              \~english Number of frames lost
    unsigned int        nNetRecvFrameCount;                         ///< [OUT] \~chinese ���յ���ͼ��֡��, GenTL GIGE�����֧�� \~english Received Frame Count, not support GenTL GiGE devices
    int64_t             nRequestResendPacketCount;                  ///< [OUT] \~chinese �����ط�����                          \~english Request Resend Packet Count
    int64_t             nResendPacketCount;                         ///< [OUT] \~chinese �ط�����                              \~english Resend Packet Count

}MV_MATCH_INFO_NET_DETECT;

/// \~chinese host�յ���u3v�豸�˵����ֽ�������Ӧ����Ϊ MV_MATCH_TYPE_USB_DETECT    \~english The total number of bytes host received from the u3v device side, the corresponding type is MV_MATCH_TYPE_USB_DETECT
typedef struct _MV_MATCH_INFO_USB_DETECT_
{
    int64_t             nReceiveDataSize;                           ///< [OUT] \~chinese �ѽ������ݴ�С [Open��Close֮��]   \~english Received data size
    unsigned int        nReceivedFrameCount;                        ///< [OUT] \~chinese ���յ���֡��                       \~english Number of frames received
    unsigned int        nErrorFrameCount;                           ///< [OUT] \~chinese ����֡��                           \~english Number of error frames

    unsigned int        nReserved[2];                               ///<       \~chinese ����                               \~english Reserved

}MV_MATCH_INFO_USB_DETECT;

/// \~chinese ��ʾ֡��Ϣ                \~english Display frame information
typedef struct _MV_DISPLAY_FRAME_INFO_
{
    void*                   hWnd;                                   ///< [IN] \~chinese ���ھ��                \~english HWND
    unsigned char*          pData;                                  ///< [IN] \~chinese ��ʾ������              \~english Data Buffer
    unsigned int            nDataLen;                               ///< [IN] \~chinese ���ݳ���                \~english Data Size
    unsigned short          nWidth;                                 ///< [IN] \~chinese ͼ���                  \~english Width
    unsigned short          nHeight;                                ///< [IN] \~chinese ͼ���                  \~english Height
    enum MvGvspPixelType    enPixelType;                            ///< [IN] \~chinese ���ظ�ʽ                \~english Pixel format

    unsigned int            enRenderMode;                             ///  [IN] \~chinese ͼ����Ⱦ��ʽ 0-Ĭ��ģʽ(Windows GDI/Linux OPENGL), 1-D3Dģʽ(Windows��Ч)   \~english Render mode 0-Default Mode 1-D3D Mode
    unsigned int            nRes[3];                                ///<      \~chinese ����                    \~english Reserved

}MV_DISPLAY_FRAME_INFO;


typedef struct _MV_DISPLAY_FRAME_INFO_EX_
{
    unsigned int            nWidth;                                 ///< [IN]   \~chinese ͼ���                \~english Width
    unsigned int            nHeight;                                ///< [IN]   \~chinese ͼ���                \~english Height
    enum MvGvspPixelType    enPixelType;                            ///< [IN]   \~chinese ���ظ�ʽ              \~english Pixel type

    unsigned char*          pImageBuf;                              ///< [IN]   \~chinese ����ͼ�񻺴�          \~english Input image buffer
    unsigned int            nImageBufLen;                           ///< [IN]   \~chinese ����ͼ�񳤶�          \~english Input image length

    unsigned int            enRenderMode;                            ///  [IN]   \~chinese ͼ����Ⱦ��ʽ 0-Ĭ��ģʽ(Windows GDI/Linux OPENGL), 1-D3Dģʽ(Windows��Ч)   \~english Render mode 0-Default Mode 1-D3D Mode
    unsigned int            nRes[3];                                ///<      \~chinese ����                    \~english Reserved

}MV_DISPLAY_FRAME_INFO_EX;

/// \~chinese �����3D���ݸ�ʽ          \~english The saved format for 3D data
enum MV_SAVE_POINT_CLOUD_FILE_TYPE
{
    MV_PointCloudFile_Undefined         = 0,                        ///< \~chinese δ����ĵ��Ƹ�ʽ             \~english Undefined point cloud format
    MV_PointCloudFile_PLY               = 1,                        ///< \~chinese PLY���Ƹ�ʽ                  \~english The point cloud format named PLY
    MV_PointCloudFile_CSV               = 2,                        ///< \~chinese CSV���Ƹ�ʽ                  \~english The point cloud format named CSV
    MV_PointCloudFile_OBJ               = 3,                        ///< \~chinese OBJ���Ƹ�ʽ                  \~english The point cloud format named OBJ

};

/// \~chinese ����3D���ݵ�����          \~english Save 3D data to buffer
typedef struct _MV_SAVE_POINT_CLOUD_PARAM_
{
    unsigned int                    nLinePntNum;                    ///< [IN]  \~chinese �е�������ͼ���       \~english The number of points in each row,which is the width of the image
    unsigned int                    nLineNum;                       ///< [IN]  \~chinese ��������ͼ���         \~english The number of rows,which is the height of the image

    enum MvGvspPixelType            enSrcPixelType;                 ///< [IN]  \~chinese �������ݵ����ظ�ʽ     \~english The pixel format of the input data
    unsigned char*                  pSrcData;                       ///< [IN]  \~chinese �������ݻ���           \~english Input data buffer
    unsigned int                    nSrcDataLen;                    ///< [IN]  \~chinese �������ݳ���           \~english Input data length

    unsigned char*                  pDstBuf;                        ///< [OUT] \~chinese ����������ݻ���       \~english Output pixel data buffer
    unsigned int                    nDstBufSize;                    ///< [IN]  \~chinese �ṩ�������������С(nLinePntNum * nLineNum * (16*3 + 4) + 2048)   \~english Output buffer size provided(nLinePntNum * nLineNum * (16*3 + 4) + 2048) 
    unsigned int                    nDstBufLen;                     ///< [OUT] \~chinese ����������ݻ��泤��   \~english Output pixel data buffer size
    MV_SAVE_POINT_CLOUD_FILE_TYPE   enPointCloudFileType;           ///< [IN]  \~chinese �ṩ����ĵ����ļ����� \~english Output point data file type provided

    unsigned int        nReserved[8];                               ///<       \~chinese �����ֶ�               \~english Reserved

}MV_SAVE_POINT_CLOUD_PARAM;

/// \~chinese ����ͼƬ��ʽ              \~english Save image type
enum MV_SAVE_IAMGE_TYPE
{
    MV_Image_Undefined                  = 0,                        ///< \~chinese δ�����ͼ���ʽ             \~english Undefined Image Type
    MV_Image_Bmp                        = 1,                        ///< \~chinese BMPͼ���ʽ                  \~english BMP Image Type
    MV_Image_Jpeg                       = 2,                        ///< \~chinese JPEGͼ���ʽ                 \~english Jpeg Image Type
    MV_Image_Png                        = 3,                        ///< \~chinese PNGͼ���ʽ                  \~english Png  Image Type
    MV_Image_Tif                        = 4,                        ///< \~chinese TIFFͼ���ʽ                 \~english TIFF Image Type

};

/// \~chinese ͼƬ�������              \~english Save Image Parameters
typedef struct _MV_SAVE_IMAGE_PARAM_EX3_
{
    unsigned char*          pData;                                  ///< [IN]  \~chinese �������ݻ���           \~english Input Data Buffer
    unsigned int            nDataLen;                               ///< [IN]  \~chinese �������ݳ���           \~english Input Data length
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese �������ݵ����ظ�ʽ     \~english Input Data Pixel Format
    unsigned int          nWidth;                                   ///< [IN]  \~chinese ͼ���                 \~english Image Width
    unsigned int          nHeight;                                  ///< [IN]  \~chinese ͼ���                 \~english Image Height

    unsigned char*          pImageBuffer;                           ///< [OUT] \~chinese ���ͼƬ����           \~english Output Image Buffer
    unsigned int            nImageLen;                              ///< [OUT] \~chinese ���ͼƬ����           \~english Output Image length
    unsigned int            nBufferSize;                            ///< [IN]  \~chinese �ṩ�������������С   \~english Output buffer size provided
    enum MV_SAVE_IAMGE_TYPE enImageType;                            ///< [IN]  \~chinese ���ͼƬ��ʽ           \~english Output Image Format
    unsigned int            nJpgQuality;                            ///< [IN]  \~chinese JPG��������(50-99]��������ʽ��Ч   \~english Encoding quality(50-99]��Other formats are invalid


    unsigned int            iMethodValue;                           ///< [IN]  \~chinese ��ֵ���� 0-���� 1-���⣨����ֵĬ��Ϊ���⣩ 2-���� 3-����+  \~english Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal 3-Optimal+

    unsigned int            nReserved[3];                           ///<       \~chinese Ԥ��                   \~english Reserved

}MV_SAVE_IMAGE_PARAM_EX3;


// ����ͼƬ���ļ�����
typedef struct _MV_SAVE_IMAGE_TO_FILE_PARAM_EX_
{
    unsigned int        nWidth;             // [IN]     ͼ���
    unsigned int        nHeight;            // [IN]     ͼ���
    MvGvspPixelType     enPixelType;        // [IN]     �������ݵ����ظ�ʽ
    unsigned char*      pData;              // [IN]     �������ݻ���
    unsigned int        nDataLen;           // [IN]     �������ݴ�С

    MV_SAVE_IAMGE_TYPE  enImageType;        // [IN]     ����ͼƬ��ʽ
    char*               pcImagePath;        // [IN]     �����ļ�·��

    unsigned int        nQuality;           // [IN]     JPG��������(50-99]��������ʽ��Ч
    int                 iMethodValue;       // [IN]     ��ֵ���� 0-���� 1-���⣨����ֵĬ��Ϊ���⣩ 2-���� 3-����+
    unsigned int        nReserved[8];

}MV_SAVE_IMAGE_TO_FILE_PARAM_EX;


/// \~chinese ��ת�Ƕ�                  \~english Rotation angle
typedef enum _MV_IMG_ROTATION_ANGLE_
{
    MV_IMAGE_ROTATE_90                  = 1,
    MV_IMAGE_ROTATE_180                 = 2,
    MV_IMAGE_ROTATE_270                 = 3,

}MV_IMG_ROTATION_ANGLE;

/// \~chinese ͼ����ת�ṹ��            \~english Rotate image structure
typedef struct _MV_CC_ROTATE_IMAGE_PARAM_T_
{
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese ���ظ�ʽ               \~english Pixel format
    unsigned int            nWidth;                                 ///< [IN][OUT] \~chinese ͼ���             \~english Width
    unsigned int            nHeight;                                ///< [IN][OUT] \~chinese ͼ���             \~english Height

    unsigned char*          pSrcData;                               ///< [IN]  \~chinese �������ݻ���           \~english Input data buffer
    unsigned int            nSrcDataLen;                            ///< [IN]  \~chinese �������ݳ���           \~english Input data length

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese ������ݻ���           \~english Output data buffer
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese ������ݳ���           \~english Output data length
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese �ṩ�������������С   \~english Provided output buffer size

    MV_IMG_ROTATION_ANGLE   enRotationAngle;                        ///< [IN]  \~chinese ��ת�Ƕ�               \~english Rotation angle

    unsigned int            nRes[8];                                ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_ROTATE_IMAGE_PARAM;

/// \~chinese ��ת����                  \~english Flip type
typedef enum _MV_IMG_FLIP_TYPE_
{
    MV_FLIP_VERTICAL                    = 1,
    MV_FLIP_HORIZONTAL                  = 2,

}MV_IMG_FLIP_TYPE;

/// \~chinese ͼ��ת�ṹ��            \~english Flip image structure
typedef struct _MV_CC_FLIP_IMAGE_PARAM_T_
{
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese ���ظ�ʽ               \~english Pixel format
    unsigned int            nWidth;                                 ///< [IN]  \~chinese ͼ���                 \~english Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese ͼ���                 \~english Height

    unsigned char*          pSrcData;                               ///< [IN]  \~chinese �������ݻ���           \~english Input data buffer
    unsigned int            nSrcDataLen;                            ///< [IN]  \~chinese �������ݳ���           \~english Input data length

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese ������ݻ���           \~english Output data buffer
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese ������ݳ���           \~english Output data length
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese �ṩ�������������С   \~english Provided output buffer size

    MV_IMG_FLIP_TYPE        enFlipType;                             ///< [IN]  \~chinese ��ת����               \~english Flip type

    unsigned int            nRes[8];                                ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_FLIP_IMAGE_PARAM;


/// \~chinese ����ת���ṹ��            \~english Pixel convert structure
typedef struct _MV_CC_PIXEL_CONVERT_PARAM_EX_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese ͼ���                 \~english Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese ͼ���                 \~english Height

    enum MvGvspPixelType    enSrcPixelType;                         ///< [IN]  \~chinese Դ���ظ�ʽ             \~english Source pixel format
    unsigned char*          pSrcData;                               ///< [IN]  \~chinese �������ݻ���           \~english Input data buffer
    unsigned int            nSrcDataLen;                            ///< [IN]  \~chinese �������ݳ���           \~english Input data length

    enum MvGvspPixelType    enDstPixelType;                         ///< [IN]  \~chinese Ŀ�����ظ�ʽ           \~english Destination pixel format
    unsigned char*          pDstBuffer;                             ///< [OUT] \~chinese ������ݻ���           \~english Output data buffer
    unsigned int            nDstLen;                                ///< [OUT] \~chinese ������ݳ���           \~english Output data length
    unsigned int            nDstBufferSize;                         ///< [IN]  \~chinese �ṩ�������������С   \~english Provided output buffer size

    unsigned int            nRes[4];                                ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_PIXEL_CONVERT_PARAM_EX;





/// \~chinese Gamma����                 \~english Gamma type
typedef enum _MV_CC_GAMMA_TYPE_
{
    MV_CC_GAMMA_TYPE_NONE               = 0,                        ///< \~chinese ������                       \~english Disable
    MV_CC_GAMMA_TYPE_VALUE              = 1,                        ///< \~chinese Gammaֵ                      \~english Gamma value
    MV_CC_GAMMA_TYPE_USER_CURVE         = 2,                        ///< \~chinese Gamma����                    \~english Gamma curve
                                                                    ///< \~chinese 8λ,���ȣ�256*sizeof(unsigned char)      \~english 8bit,length:256*sizeof(unsigned char)
                                                                    ///< \~chinese 10λ,���ȣ�1024*sizeof(unsigned short)   \~english 10bit,length:1024*sizeof(unsigned short)
                                                                    ///< \~chinese 12λ,���ȣ�4096*sizeof(unsigned short)   \~english 12bit,length:4096*sizeof(unsigned short)
                                                                    ///< \~chinese 16λ,���ȣ�65536*sizeof(unsigned short)  \~english 16bit,length:65536*sizeof(unsigned short)
    MV_CC_GAMMA_TYPE_LRGB2SRGB          = 3,                        ///< \~chinese linear RGB to sRGB           \~english linear RGB to sRGB
    MV_CC_GAMMA_TYPE_SRGB2LRGB          = 4,                        ///< \~chinese sRGB to linear RGB(��ɫ�ʲ�ֵʱ֧�֣�ɫ��У��ʱ��Ч) \~english sRGB to linear RGB

}MV_CC_GAMMA_TYPE;

// Gamma��Ϣ
/// \~chinese Gamma��Ϣ�ṹ��           \~english Gamma info structure
typedef struct _MV_CC_GAMMA_PARAM_T_
{
    MV_CC_GAMMA_TYPE    enGammaType;                                ///< [IN]  \~chinese Gamma����              \~english Gamma type
    float               fGammaValue;                                ///< [IN]  \~chinese Gammaֵ:0.1 ~ 4.0       \~english Gamma value:0.1 ~ 4.0
    unsigned char*      pGammaCurveBuf;                             ///< [IN]  \~chinese Gamma���߻���          \~english Gamma curve buffer
    unsigned int        nGammaCurveBufLen;                          ///< [IN]  \~chinese Gamma���߳���          \~english Gamma curve buffer size

    unsigned int        nRes[8];                                    ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_GAMMA_PARAM;

/// \~chinese CCM����                   \~english CCM param
typedef struct _MV_CC_CCM_PARAM_T_
{
    bool                bCCMEnable;                                 ///< [IN]  \~chinese �Ƿ�����CCM            \~english CCM enable
    int                 nCCMat[9];                                  ///< [IN]  \~chinese CCM����[-8192~8192]    \~english Color correction matrix[-8192~8192]

    unsigned int        nRes[8];                                    ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_CCM_PARAM;

/// \~chinese CCM����                   \~english CCM param
typedef struct _MV_CC_CCM_PARAM_EX_T_
{
    bool                bCCMEnable;                                 ///< [IN]  \~chinese �Ƿ�����CCM            \~english CCM enable
    int                 nCCMat[9];                                  ///< [IN]  \~chinese CCM����[-65536~65536]  \~english Color correction matrix[-65536~65536]
    unsigned int        nCCMScale;                                  ///< [IN]  \~chinese ����ϵ����2��������,���65536��    \~english Quantitative scale(Integer power of 2, <= 65536)

    unsigned int        nRes[8];                                    ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_CCM_PARAM_EX;

/// \~chinese �Աȶȵ��ڽṹ��          \~english Contrast structure
typedef struct _MV_CC_CONTRAST_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese ͼ����(��С8)        \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese ͼ��߶�(��С8)        \~english Image Height
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese �������ݻ���           \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese �������ݴ�С           \~english Input data length
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese ���ظ�ʽ               \~english Pixel format

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese ������ݻ���           \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese �ṩ�������������С   \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese ������ݳ���           \~english Output data length

    unsigned int            nContrastFactor;                        ///< [IN]  \~chinese �Աȶ�ֵ��[1,10000]     \~english Contrast factor,[1,10000]

    unsigned int            nRes[8];                                ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_CONTRAST_PARAM;

/// \~chinese ˮӡ��Ϣ                  \~english  Frame-specific information
typedef struct _MV_CC_FRAME_SPEC_INFO_
{
    /// \~chinese �豸ˮӡʱ��      \~english Device frame-specific time scale
    unsigned int        nSecondCount;                               ///< [OUT] \~chinese ����                   \~english The Seconds
    unsigned int        nCycleCount;                                ///< [OUT] \~chinese ������                 \~english The Count of Cycle
    unsigned int        nCycleOffset;                               ///< [OUT] \~chinese ����ƫ����             \~english The Offset of Cycle

    float               fGain;                                      ///< [OUT] \~chinese ����                   \~english Gain
    float               fExposureTime;                              ///< [OUT] \~chinese �ع�ʱ��               \~english Exposure Time
    unsigned int        nAverageBrightness;                         ///< [OUT] \~chinese ƽ������               \~english Average brightness

    /// \~chinese ��ƽ�����        \~english White balance
    unsigned int        nRed;                                       ///< [OUT] \~chinese ��ɫ                   \~english Red
    unsigned int        nGreen;                                     ///< [OUT] \~chinese ��ɫ                   \~english Green
    unsigned int        nBlue;                                      ///< [OUT] \~chinese ��ɫ                   \~english Blue

    unsigned int        nFrameCounter;                              ///< [OUT] \~chinese ��֡��                 \~english Frame Counter
    unsigned int        nTriggerIndex;                              ///< [OUT] \~chinese ��������               \~english Trigger Counting

    unsigned int        nInput;                                     ///< [OUT] \~chinese ����                   \~english Input
    unsigned int        nOutput;                                    ///< [OUT] \~chinese ���                   \~english Output

    /// \~chinese ROI����           \~english ROI Region
    unsigned short      nOffsetX;                                   ///< [OUT] \~chinese ˮƽƫ����             \~english OffsetX
    unsigned short      nOffsetY;                                   ///< [OUT] \~chinese ��ֱƫ����             \~english OffsetY
    unsigned short      nFrameWidth;                                ///< [OUT] \~chinese ˮӡ��                 \~english The Width of Chunk
    unsigned short      nFrameHeight;                               ///< [OUT] \~chinese ˮӡ��                 \~english The Height of Chunk

    unsigned int        nReserved[16];                              ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_FRAME_SPEC_INFO;

/// \~chinese ����������              \~english High Bandwidth decode structure
typedef struct _MV_CC_HB_DECODE_PARAM_T_
{
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese �������ݻ���           \~english Input data buffer
    unsigned int            nSrcLen;                                ///< [IN]  \~chinese �������ݴ�С           \~english Input data size

    unsigned int            nWidth;                                 ///< [OUT] \~chinese ͼ���                 \~english Width
    unsigned int            nHeight;                                ///< [OUT] \~chinese ͼ���                 \~english Height
    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese ������ݻ���           \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese �ṩ�������������С   \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese ������ݴ�С           \~english Output data size
    enum MvGvspPixelType    enDstPixelType;                         ///< [OUT] \~chinese ��������ظ�ʽ         \~english Output pixel format

    MV_CC_FRAME_SPEC_INFO   stFrameSpecInfo;                        ///< [OUT] \~chinese ˮӡ��Ϣ               \~english Frame Spec Info

    unsigned int            nRes[8];                                ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_HB_DECODE_PARAM;

/// \~chinese ¼���ʽ����              \~english Record Format Type
typedef enum _MV_RECORD_FORMAT_TYPE_
{
    MV_FormatType_Undefined             = 0,                        ///< \~chinese δ�����¼���ʽ             \~english Undefined Recode Format Type
    MV_FormatType_AVI                   = 1,                        ///< \~chinese AVI¼���ʽ                  \~english AVI Recode Format Type

}MV_RECORD_FORMAT_TYPE;

/// \~chinese ¼�����                  \~english Record Parameters
typedef struct _MV_CC_RECORD_PARAM_T_
{
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese �������ݵ����ظ�ʽ     \~english Pixel Type

    unsigned short          nWidth;                                 ///< [IN]  \~chinese ͼ���(2�ı���)        \~english Width
    unsigned short          nHeight;                                ///< [IN]  \~chinese ͼ���(2�ı���)        \~english Height

    float                   fFrameRate;                             ///< [IN]  \~chinese ֡��fps(����1/16)      \~english The Rate of Frame
    unsigned int            nBitRate;                               ///< [IN]  \~chinese ����kbps(128-16*1024)  \~english The Rate of Bitrate

    MV_RECORD_FORMAT_TYPE   enRecordFmtType;                        ///< [IN]  \~chinese ¼���ʽ               \~english Recode Format Type

    char*                   strFilePath;                            ///< [IN]  \~chinese ¼���ļ����·��(���·���д������ģ���ת��utf-8)  \~english File Path

    unsigned int            nRes[8];                                ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_RECORD_PARAM;

/// \~chinese �����ͼ������            \~english Input Data
typedef struct _MV_CC_INPUT_FRAME_INFO_T_
{
    unsigned char*      pData;                                      ///< [IN]  \~chinese ͼ������ָ��           \~english Record Data
    unsigned int        nDataLen;                                   ///< [IN]  \~chinese ͼ���С               \~english The Length of Record Data

    unsigned int        nRes[8];                                    ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_INPUT_FRAME_INFO;

/// \~chinese �ɼ�ģʽ                  \~english Acquisition mode
typedef enum _MV_CAM_ACQUISITION_MODE_
{
    MV_ACQ_MODE_SINGLE                  = 0,                        ///< \~chinese ��֡ģʽ                     \~english Single Mode
    MV_ACQ_MODE_MUTLI                   = 1,                        ///< \~chinese ��֡ģʽ                     \~english Multi Mode
    MV_ACQ_MODE_CONTINUOUS              = 2,                        ///< \~chinese �����ɼ�ģʽ                 \~english Continuous Mode

}MV_CAM_ACQUISITION_MODE;

/// \~chinese ����ģʽ                  \~english Gain Mode
typedef enum _MV_CAM_GAIN_MODE_
{
    MV_GAIN_MODE_OFF                    = 0,                        ///< \~chinese �ر�                         \~english Single Mode
    MV_GAIN_MODE_ONCE                   = 1,                        ///< \~chinese һ��                         \~english Multi Mode
    MV_GAIN_MODE_CONTINUOUS             = 2,                        ///< \~chinese ����                         \~english Continuous Mode

}MV_CAM_GAIN_MODE;

/// \~chinese �ع�ģʽ                  \~english Exposure Mode
typedef enum _MV_CAM_EXPOSURE_MODE_
{
    MV_EXPOSURE_MODE_TIMED              = 0,                        ///< \~chinese ʱ��                         \~english Timed
    MV_EXPOSURE_MODE_TRIGGER_WIDTH      = 1,                        ///< \~chinese ����������                 \~english TriggerWidth
}MV_CAM_EXPOSURE_MODE;

/// \~chinese �Զ��ع�ģʽ              \~english Auto Exposure Mode
typedef enum _MV_CAM_EXPOSURE_AUTO_MODE_
{
    MV_EXPOSURE_AUTO_MODE_OFF           = 0,                        ///< \~chinese �ر�                         \~english Off
    MV_EXPOSURE_AUTO_MODE_ONCE          = 1,                        ///< \~chinese һ��                         \~english Once
    MV_EXPOSURE_AUTO_MODE_CONTINUOUS    = 2,                        ///< \~chinese ����                         \~english Continuous

}MV_CAM_EXPOSURE_AUTO_MODE;

/// \~chinese ����ģʽ                  \~english Trigger Mode
typedef enum _MV_CAM_TRIGGER_MODE_
{
    MV_TRIGGER_MODE_OFF                 = 0,                        ///< \~chinese �ر�                         \~english Off
    MV_TRIGGER_MODE_ON                  = 1,                        ///< \~chinese ��                         \~english ON

}MV_CAM_TRIGGER_MODE;

/// \~chinese Gammaѡ����               \~english Gamma Selector
typedef enum _MV_CAM_GAMMA_SELECTOR_
{
    MV_GAMMA_SELECTOR_USER              = 1,                        ///< \~chinese �û�                         \~english Gamma Selector User
    MV_GAMMA_SELECTOR_SRGB              = 2,                        ///< \~chinese sRGB                         \~english Gamma Selector sRGB

}MV_CAM_GAMMA_SELECTOR;

/// \~chinese ��ƽ��                    \~english White Balance
typedef enum _MV_CAM_BALANCEWHITE_AUTO_
{
    MV_BALANCEWHITE_AUTO_OFF            = 0,                        ///< \~chinese �ر�                         \~english Off
    MV_BALANCEWHITE_AUTO_ONCE           = 2,                        ///< \~chinese һ��                         \~english Once
    MV_BALANCEWHITE_AUTO_CONTINUOUS     = 1,                        ///< \~chinese ����                         \~english Continuous

}MV_CAM_BALANCEWHITE_AUTO;

/// \~chinese ����Դ                    \~english Trigger Source
typedef enum _MV_CAM_TRIGGER_SOURCE_
{
    MV_TRIGGER_SOURCE_LINE0             = 0,                        ///< \~chinese Line0                        \~english Line0
    MV_TRIGGER_SOURCE_LINE1             = 1,                        ///< \~chinese Line1                        \~english Line1
    MV_TRIGGER_SOURCE_LINE2             = 2,                        ///< \~chinese Line2                        \~english Line2
    MV_TRIGGER_SOURCE_LINE3             = 3,                        ///< \~chinese Line3                        \~english Line3
    MV_TRIGGER_SOURCE_COUNTER0          = 4,                        ///< \~chinese Conuter0                     \~english Conuter0

    MV_TRIGGER_SOURCE_SOFTWARE          = 7,                        ///< \~chinese ����                       \~english Software
    MV_TRIGGER_SOURCE_FrequencyConverter= 8,                        ///< \~chinese ��Ƶ��                       \~english Frequency Converter

}MV_CAM_TRIGGER_SOURCE;

/// \~chinese GigEVision IP����         \~english GigEVision IP Configuration
#define MV_IP_CFG_STATIC                0x05000000                  ///< \~chinese ��̬                         \~english Static
#define MV_IP_CFG_DHCP                  0x06000000                  ///< \~chinese DHCP                         \~english DHCP
#define MV_IP_CFG_LLA                   0x04000000                  ///< \~chinese LLA                          \~english LLA

/// \~chinese GigEVision���紫��ģʽ    \~english GigEVision Net Transfer Mode
#define MV_NET_TRANS_DRIVER             0x00000001                  ///< \~chinese ����                         \~english Driver
#define MV_NET_TRANS_SOCKET             0x00000002                  ///< \~chinese Socket                       \~english Socket

/// \~chinese CameraLink������          \~english CameraLink Baud Rates (CLUINT32)
#define MV_CAML_BAUDRATE_9600           0x00000001                  ///< \~chinese 9600                         \~english 9600
#define MV_CAML_BAUDRATE_19200          0x00000002                  ///< \~chinese 19200                        \~english 19200
#define MV_CAML_BAUDRATE_38400          0x00000004                  ///< \~chinese 38400                        \~english 38400
#define MV_CAML_BAUDRATE_57600          0x00000008                  ///< \~chinese 57600                        \~english 57600
#define MV_CAML_BAUDRATE_115200         0x00000010                  ///< \~chinese 115200                       \~english 115200
#define MV_CAML_BAUDRATE_230400         0x00000020                  ///< \~chinese 230400                       \~english 230400
#define MV_CAML_BAUDRATE_460800         0x00000040                  ///< \~chinese 460800                       \~english 460800
#define MV_CAML_BAUDRATE_921600         0x00000080                  ///< \~chinese 921600                       \~english 921600
#define MV_CAML_BAUDRATE_AUTOMAX        0x40000000                  ///< \~chinese ���ֵ                       \~english Auto Max

/// \~chinese �쳣��Ϣ����              \~english Exception message type
#define MV_EXCEPTION_DEV_DISCONNECT     0x00008001                  ///< \~chinese �豸�Ͽ�����                 \~english The device is disconnected
#define MV_EXCEPTION_VERSION_CHECK      0x00008002                  ///< \~chinese SDK�������汾��ƥ��          \~english SDK does not match the driver version

/// \~chinese U3V���쳣����
typedef enum _MV_CC_STREAM_EXCEPTION_TYPE_
{
    MV_CC_STREAM_EXCEPTION_ABNORMAL_IMAGE   = 0x4001,               ///< \~chinese �쳣��ͼ�񣬸�֡������
    MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW    = 0x4002,               ///< \~chinese �����б�����������ɵ�һ֡
    MV_CC_STREAM_EXCEPTION_LIST_EMPTY       = 0x4003,               ///< \~chinese �����б�Ϊ�գ���֡������
    MV_CC_STREAM_EXCEPTION_RECONNECTION     = 0x4004,               ///< \~chinese �����ָ�
    MV_CC_STREAM_EXCEPTION_DISCONNECTED     = 0x4005,               ///< \~chinese ����,�ָ�ʧ��,ȡ������ֹ
    MV_CC_STREAM_EXCEPTION_DEVICE           = 0x4006,               ///< \~chinese �豸�쳣,ȡ������ֹ

}MV_CC_STREAM_EXCEPTION_TYPE;

///< \~chinese �豸Event�¼�������󳤶�    \~english Max length of event name
#define MAX_EVENT_NAME_SIZE             128

/// \~chinese Event�¼��ص���Ϣ\        \~english Event callback infomation
typedef struct _MV_EVENT_OUT_INFO_
{
    char                EventName[MAX_EVENT_NAME_SIZE];             ///< [OUT] \~chinese Event����              \~english Event name

    unsigned short      nEventID;                                   ///< [OUT] \~chinese Event��                \~english Event ID
    unsigned short      nStreamChannel;                             ///< [OUT] \~chinese ��ͨ�����             \~english Circulation number

    unsigned int        nBlockIdHigh;                               ///< [OUT] \~chinese ֡�Ÿ�λ   (���޹̼�֧��)           \~english BlockId high, not support
    unsigned int        nBlockIdLow;                                ///< [OUT] \~chinese ֡�ŵ�λ   (���޹̼�֧��)           \~english BlockId low, not support

    unsigned int        nTimestampHigh;                             ///< [OUT] \~chinese ʱ�����λ             \~english Timestramp high
    unsigned int        nTimestampLow;                              ///< [OUT] \~chinese ʱ�����λ             \~english Timestramp low

    void*               pEventData;                                 ///< [OUT] \~chinese Event����     (���޹̼�֧��)        \~english Event data, not support
    unsigned int        nEventDataSize;                             ///< [OUT] \~chinese Event���ݳ��� (���޹̼�֧��)        \~english Event data len, not support

    unsigned int        nReserved[16];                              ///<       \~chinese Ԥ��                   \~english Reserved

}MV_EVENT_OUT_INFO;

/// \~chinese �ļ���ȡ                  \~english File Access
typedef struct _MV_CC_FILE_ACCESS_T
{
    const char*         pUserFileName;                              ///< [IN]  \~chinese �û��ļ���             \~english User file name
    const char*         pDevFileName;                               ///< [IN]  \~chinese �豸�ļ���             \~english Device file name

    unsigned int        nReserved[32];                              ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_FILE_ACCESS;

/// \~chinese �ļ���ȡ                  \~english File Access
typedef struct _MV_CC_FILE_ACCESS_E
{
    char*               pUserFileBuf;                              ///< [IN]       \~chinese �û����ݻ���                                                                 \~english User data Buffer
    unsigned int        pFileBufSize;                              ///< [IN]       \~chinese �û����ݻ����С                                                             \~english data buffer size
    unsigned int        pFileBufLen;                               ///< [OUT][IN]  \~chinese �ļ����ݻ����ܳ���(��ȡʱ����ļ������ܳ���,д��ʱ�����ļ������ܳ���)          \~english data buffer len
    const char*         pDevFileName;                              ///< [IN]       \~chinese �豸�ļ���                                                                    \~english Device file name

    unsigned int        nReserved[32];                              ///<           \~chinese Ԥ��                                                                          \~english Reserved
}MV_CC_FILE_ACCESS_EX;

/// \~chinese �ļ���ȡ����              \~english File Access Progress
typedef struct _MV_CC_FILE_ACCESS_PROGRESS_T
{
    int64_t             nCompleted;                                 ///< [OUT] \~chinese ����ɵĳ���           \~english Completed Length
    int64_t             nTotal;                                     ///< [OUT] \~chinese �ܳ���                 \~english Total Length

    unsigned int        nReserved[8];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MV_CC_FILE_ACCESS_PROGRESS;

/// \~chinese Gige�Ĵ�������            \~english The transmission type of Gige
typedef enum _MV_GIGE_TRANSMISSION_TYPE_
{
    MV_GIGE_TRANSTYPE_UNICAST               = 0x0,                  ///< \~chinese ��ʾ����(Ĭ��)                              \~english Unicast mode
    MV_GIGE_TRANSTYPE_MULTICAST             = 0x1,                  ///< \~chinese ��ʾ�鲥(�鲥IP��Χ[224.*.*.*-239.*.*.*])   \~english Multicast mode
    MV_GIGE_TRANSTYPE_LIMITEDBROADCAST      = 0x2,                  ///< \~chinese ��ʾ�������ڹ㲥���ݲ�֧��               \~english Limited broadcast mode,not support
    MV_GIGE_TRANSTYPE_SUBNETBROADCAST       = 0x3,                  ///< \~chinese ��ʾ�����ڹ㲥���ݲ�֧��                 \~english Subnet broadcast mode,not support
    MV_GIGE_TRANSTYPE_CAMERADEFINED         = 0x4,                  ///< \~chinese ��ʾ���豸��ȡ���ݲ�֧��                 \~english Transtype from camera,not support
    MV_GIGE_TRANSTYPE_UNICAST_DEFINED_PORT  = 0x5,                  ///< \~chinese ��ʾ�û��Զ���Ӧ�ö˽���ͼ������Port��   \~english User Defined Receive Data Port
    MV_GIGE_TRANSTYPE_UNICAST_WITHOUT_RECV  = 0x00010000,           ///< \~chinese ��ʾ�����˵���������ʵ��������ͼ������   \~english Unicast without receive data
    MV_GIGE_TRANSTYPE_MULTICAST_WITHOUT_RECV= 0x00010001,           ///< \~chinese ��ʾ�鲥ģʽ������ʵ��������ͼ������     \~english Multicast without receive data

}MV_GIGE_TRANSMISSION_TYPE;

/// \~chinese ���紫��ģʽ              \~english Transmission type
typedef struct _MV_TRANSMISSION_TYPE_T
{
    MV_GIGE_TRANSMISSION_TYPE   enTransmissionType;                 ///< [IN]  \~chinese ����ģʽ                   \~english Transmission type
    unsigned int                nDestIp;                            ///< [IN]  \~chinese Ŀ��IP���鲥ģʽ��������   \~english Destination IP
    unsigned short              nDestPort;                          ///< [IN]  \~chinese Ŀ��Port���鲥ģʽ�������� \~english Destination port

    unsigned int                nReserved[32];                      ///<       \~chinese Ԥ��                       \~english Reserved

}MV_TRANSMISSION_TYPE;

/// \~chinese ����������Ϣ              \~english Action Command
typedef struct _MV_ACTION_CMD_INFO_T
{
    unsigned int        nDeviceKey;                                 ///< [IN]  \~chinese �豸��Կ                                   \~english Device Key;
    unsigned int        nGroupKey;                                  ///< [IN]  \~chinese ���                                       \~english Group Key
    unsigned int        nGroupMask;                                 ///< [IN]  \~chinese ������                                     \~english Group Mask

    unsigned int        bActionTimeEnable;                          ///< [IN]  \~chinese ֻ�����ó�1ʱAction Time����Ч����1ʱ��Ч  \~english Action Time Enable
    int64_t             nActionTime;                                ///< [IN]  \~chinese Ԥ����ʱ�䣬����Ƶ�й�                     \~english Action Time

    const char*         pBroadcastAddress;                          ///< [IN]  \~chinese �㲥����ַ                                 \~english Broadcast Address
    unsigned int        nTimeOut;                                   ///< [IN]  \~chinese �ȴ�ACK�ĳ�ʱʱ�䣬���Ϊ0��ʾ����ҪACK    \~english TimeOut

    unsigned int        bSpecialNetEnable;                          ///< [IN]  \~chinese ֻ�����ó�1ʱָ��������IP����Ч����1ʱ��Ч \~english Special IP Enable
    unsigned int        nSpecialNetIP;                              ///< [IN]  \~chinese ָ��������IP                               \~english Special Net IP address

    unsigned int        nReserved[14];                              ///<       \~chinese Ԥ��                                       \~english Reserved

}MV_ACTION_CMD_INFO;

/// \~chinese �����������Ϣ          \~english Action Command Result
typedef struct _MV_ACTION_CMD_RESULT_T
{
    unsigned char       strDeviceAddress[12 + 3 + 1];               ///< [OUT] \~chinese �豸IP                 \~english IP address of the device

    int                 nStatus;                                    ///< [OUT] \~chinese ״̬��                 \~english status code returned by the device
                                                                    //1.0x0000:success.
                                                                    //2.0x8001:Command is not supported by the device.
                                                                    //3.0x8013:The device is not synchronized to a master clock to be used as time reference.
                                                                    //4.0x8015:A device queue or packet data has overflowed.
                                                                    //5.0x8016:The requested scheduled action command was requested at a time that is already past.

    unsigned int        nReserved[4];                               ///<      \~chinese Ԥ��                    \~english Reserved

}MV_ACTION_CMD_RESULT;

/// \~chinese �����������Ϣ�б�      \~english Action Command Result List
typedef struct _MV_ACTION_CMD_RESULT_LIST_T
{
    unsigned int            nNumResults;                            ///< [OUT] \~chinese ����ֵ����             \~english Number of returned values
    MV_ACTION_CMD_RESULT*   pResults;                               ///< [OUT] \~chinese ����������           \~english Reslut of action command

}MV_ACTION_CMD_RESULT_LIST;

/// \~chinese ÿ���ڵ��Ӧ�Ľӿ�����    \~english Interface type corresponds to each node 
enum MV_XML_InterfaceType
{
    IFT_IValue,                                                     ///< \~chinese Value                        \~english IValue interface
    IFT_IBase,                                                      ///< \~chinese Base                         \~english IBase interface
    IFT_IInteger,                                                   ///< \~chinese Integer                      \~english IInteger interface
    IFT_IBoolean,                                                   ///< \~chinese Boolean                      \~english IBoolean interface
    IFT_ICommand,                                                   ///< \~chinese Command                      \~english ICommand interface
    IFT_IFloat,                                                     ///< \~chinese Float                        \~english IFloat interface
    IFT_IString,                                                    ///< \~chinese String                       \~english IString interface
    IFT_IRegister,                                                  ///< \~chinese Register                     \~english IRegister interface
    IFT_ICategory,                                                  ///< \~chinese Category                     \~english ICategory interface
    IFT_IEnumeration,                                               ///< \~chinese Enumeration                  \~english IEnumeration interface
    IFT_IEnumEntry,                                                 ///< \~chinese EnumEntry                    \~english IEnumEntry interface
    IFT_IPort,                                                      ///< \~chinese Port                         \~english IPort interface
};

/// \~chinese �ڵ�ķ���ģʽ            \~english Node Access Mode
enum MV_XML_AccessMode
{
    AM_NI,                                                          ///< \~chinese ����ʵ��                     \~english Not implemented
    AM_NA,                                                          ///< \~chinese ������                       \~english Not available
    AM_WO,                                                          ///< \~chinese ֻд                         \~english Write Only
    AM_RO,                                                          ///< \~chinese ֻ��                         \~english Read Only
    AM_RW,                                                          ///< \~chinese ��д                         \~english Read and Write
    AM_Undefined,                                                   ///< \~chinese δ����                       \~english Object is not yet initialized
    AM_CycleDetect,                                                 ///< \~chinese �ڲ�����AccessModeѭ�����   \~english used internally for AccessMode cycle detection
};

/// \~chinese ���ڵ����            \~english Max Number of Nodes
#define MV_MAX_NODE_NUM             256
/// \~chinese �ڵ����Ƶ���󳤶�      \~english Max Length of a Node Name
#define MV_MAX_NODE_NAME_LEN        64
/// \~chinese �ڵ�����               \~english Node Name
typedef struct _MVCC_NODE_NAME_T
{
    char                strName[MV_MAX_NODE_NAME_LEN];              ///< \~chinese �ڵ�����                     \~english Nodes Name

    unsigned int        nReserved[4];                               ///< \~chinese Ԥ��                         \~english Reserved

}MVCC_NODE_NAME;
/// \~chinese �ڵ��б�               \~english Node List
typedef struct _MVCC_NODE_NAME_LIST_T
{
    unsigned int        nNodeNum;                                   ///< \~chinese �ڵ����                     \~english Number of Node
    MVCC_NODE_NAME      stNodeName[MV_MAX_NODE_NUM];                ///< \~chinese �ڵ�����                     \~english Node Name

    unsigned int        nReserved[4];                               ///< \~chinese Ԥ��                         \~english Reserved

}MVCC_NODE_NAME_LIST;

/// \~chinese ���������            \~english Max Number of Error
#define MV_MAX_NODE_ERROR_NUM       64

/// \~chinese �����������ʱ��ԭ��,������    \~english Reasons for importing parameter errors code
typedef enum _MVCC_NODE_ERR_TYPE_
{
    MVCC_NODE_ERR_NODE_INVALID  = 1,         ///< \~chinese �ڵ㲻����       \~english Usually, the operating node does not exist in the device
    MVCC_NODE_ERR_ACCESS        = 2,         ///< \~chinese ������������,ͨ���ǽڵ㲻�ɶ�д      \~english Access condition error, usually due to nodes not being readable or writable
    MVCC_NODE_ERR_OUT_RANGE     = 3,         ///< \~chinese д��Խ��,�����ýڵ�֧�ֵķ�Χ        \~english Write out of bounds, beyond the supported range of this node
    MVCC_NODE_ERR_VERIFY_FAILD  = 4,         ///< \~chinese У��ʧ��,ͨ����д���ֵ���ļ��е�ֵ��ƥ��        \~english Verification failed, usually due to a mismatch between the written value and the value in the file

    MVCC_NODE_ERR_OTHER         = 100,       ///< \~chinese ��������,�ɲ�����־                      \~english Other errors, can view logs

}MVCC_NODE_ERR_TYPE;
/// \~chinese ������Ϣ               \~english Error Name
typedef struct _MVCC_NODE_ERROR_T
{
    char                strName[MV_MAX_NODE_NAME_LEN];              ///< \~chinese �ڵ�����                     \~english Nodes Name
    MVCC_NODE_ERR_TYPE  enErrType;									///< \~chinese ��������                     \~english Error Type

    unsigned int        nReserved[4];                               ///< \~chinese Ԥ��                         \~english Reserved

}MVCC_NODE_ERROR;
/// \~chinese ������Ϣ�б�               \~english Error List
typedef struct _MVCC_NODE_ERROR_LIST_T
{
    unsigned int        nErrorNum;                                  ///< \~chinese �������                     \~english Number of Error
    MVCC_NODE_ERROR     stNodeError[MV_MAX_NODE_ERROR_NUM];         ///< \~chinese ������Ϣ                     \~english Error Name

    unsigned int        nReserved[4];                               ///< \~chinese Ԥ��                         \~english Reserved

}MVCC_NODE_ERROR_LIST;

/// \~chinese ���XML������             \~english Max XML Symbolic Number 
#define MV_MAX_XML_SYMBOLIC_NUM         64
/// \~chinese ö������ֵ                \~english Enumeration Value
typedef struct _MVCC_ENUMVALUE_T
{
    unsigned int        nCurValue;                                  ///< [OUT] \~chinese ��ǰֵ                 \~english Current Value
    unsigned int        nSupportedNum;                              ///< [OUT] \~chinese ���ݵ���Ч���ݸ���     \~english Number of valid data
    unsigned int        nSupportValue[MV_MAX_XML_SYMBOLIC_NUM];     ///< [OUT] \~chinese ֧�ֵ�ö��ֵ           \~english Support Value 

    unsigned int        nReserved[4];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MVCC_ENUMVALUE;

/// \~chinese ���ö����Ŀ��Ӧ�ķ��ų���            \~english Max Enum Entry Symbolic Number 
#define MV_MAX_SYMBOLIC_LEN         64
/// \~chinese ö��������Ŀ                          \~english Enumeration Entry
typedef struct _MVCC_ENUMENTRY_T
{
    unsigned int        nValue;                                     ///< [IN]  \~chinese ָ��ֵ                 \~english Value
    char                chSymbolic[MV_MAX_SYMBOLIC_LEN];            ///< [OUT] \~chinese ָ��ֵ��Ӧ�ķ���       \~english Symbolic

    unsigned int        nReserved[4];                               ///< \~chinese Ԥ��                         \~english Reserved

}MVCC_ENUMENTRY;

/// \~chinese Int����ֵ                 \~english Int Value
typedef struct _MVCC_INTVALUE_T
{
    unsigned int        nCurValue;                                  ///< [OUT] \~chinese ��ǰֵ                 \~english Current Value
    unsigned int        nMax;                                       ///< [OUT] \~chinese ���ֵ                 \~english Max
    unsigned int        nMin;                                       ///< [OUT] \~chinese ��Сֵ                 \~english Min
    unsigned int        nInc;                                       ///< [OUT] \~chinese                        \~english Inc

    unsigned int        nReserved[4];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MVCC_INTVALUE;

/// \~chinese Int����ֵEx               \~english Int Value Ex
typedef struct _MVCC_INTVALUE_EX_T
{
    int64_t             nCurValue;                                  ///< [OUT] \~chinese ��ǰֵ                 \~english Current Value
    int64_t             nMax;                                       ///< [OUT] \~chinese ���ֵ                 \~english Max
    int64_t             nMin;                                       ///< [OUT] \~chinese ��Сֵ                 \~english Min
    int64_t             nInc;                                       ///< [OUT] \~chinese Inc                    \~english Inc

    unsigned int        nReserved[16];                              ///<       \~chinese Ԥ��                   \~english Reserved

}MVCC_INTVALUE_EX;

/// \~chinese Float����ֵ               \~english Float Value
typedef struct _MVCC_FLOATVALUE_T
{
    float               fCurValue;                                  ///< [OUT] \~chinese ��ǰֵ                 \~english Current Value
    float               fMax;                                       ///< [OUT] \~chinese ���ֵ                 \~english Max
    float               fMin;                                       ///< [OUT] \~chinese ��Сֵ                 \~english Min

    unsigned int        nReserved[4];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MVCC_FLOATVALUE;

/// \~chinese String����ֵ              \~english String Value
typedef struct _MVCC_STRINGVALUE_T
{
    char                chCurValue[256];                            ///< [OUT] \~chinese ��ǰֵ                 \~english Current Value

    int64_t             nMaxLength;                                 ///< [OUT] \~chinese ��󳤶�               \~english MaxLength
    unsigned int        nReserved[2];                               ///<       \~chinese Ԥ��                   \~english Reserved

}MVCC_STRINGVALUE;

/// \~chinese ��������ɫ                \~english Color of Auxiliary Line
typedef struct _MVCC_COLORF
{
	float           fR;             ///< [IN] \~chinese ��ɫ������������ɫ�������ȣ���ΧΪ[0.0 , 1.0]��������[0, 255]����ɫ���   \~english Red��Range[0.0, 1.0]
    float           fG;             ///< [IN] \~chinese ��ɫ������������ɫ�������ȣ���ΧΪ[0.0 , 1.0]��������[0, 255]����ɫ���   \~english Green��Range[0.0, 1.0]
    float           fB;             ///< [IN] \~chinese ��ɫ������������ɫ�������ȣ���ΧΪ[0.0 , 1.0]��������[0, 255]����ɫ���   \~english Blue��Range[0.0, 1.0]
    float           fAlpha;         ///< [IN] \~chinese ͸���ȣ�����������ɫ�����͸���ȣ���ΧΪ[0.0 , 1.0] (�˲��������ݲ�֧��)    \~english Alpha��Range[0.0, 1.0](Not Support)
    unsigned int    nReserved[4];   ///<      \~chinese Ԥ��                        \~english Reserved

}MVCC_COLORF;

/// \~chinese �Զ����                    \~english Point defined
typedef struct _MVCC_POINTF
{
    float           fX;             ///< [IN] \~chinese �õ����ͼ�����Ե���룬����ͼ������λ�ã���ΧΪ[0.0 , 1.0]   \~english Distance From Left��Range[0.0, 1.0]
    float           fY;             ///< [IN] \~chinese �õ����ͼ���ϱ�Ե���룬����ͼ������λ�ã���ΧΪ[0.0 , 1.0]   \~english Distance From Top��Range[0.0, 1.0]
    unsigned int    nReserved[4];   ///<      \~chinese Ԥ��                        \~english Reserved

}MVCC_POINTF;

/// \~chinese ���ο�������Ϣ            \~english Rect Area Info
typedef struct _MVCC_RECT_INFO
{
    float           fTop;           ///< [IN] \~chinese �����ϱ�Ե����ͼ���ϱ�Ե�ľ��룬����ͼ������λ�ã���ΧΪ[0.0 , 1.0]   \~english Distance From Top��Range[0, 1.0]
    float           fBottom;        ///< [IN] \~chinese �����±�Ե����ͼ���ϱ�Ե�ľ��룬����ͼ������λ�ã���ΧΪ[0.0 , 1.0]   \~english Distance From Top��Range[0, 1.0]
    float           fLeft;          ///< [IN] \~chinese �������Ե����ͼ�����Ե�ľ��룬����ͼ������λ�ã���ΧΪ[0.0 , 1.0]   \~english Distance From Left��Range[0, 1.0]
    float           fRight;         ///< [IN] \~chinese �����ұ�Ե����ͼ�����Ե�ľ��룬����ͼ������λ�ã���ΧΪ[0.0 , 1.0]   \~english Distance From Left��Range[0, 1.0]

    MVCC_COLORF     stColor;        ///< [IN] \~chinese ��������ɫ                      \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///< [IN] \~chinese �����߿�ȣ����ֻ����1��2      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<      \~chinese Ԥ��                            \~english Reserved

}MVCC_RECT_INFO;

/// \~chinese Բ�ο�������Ϣ            \~english Circle Area Info
typedef struct _MVCC_CIRCLE_INFO
{
    MVCC_POINTF     stCenterPoint;  ///< [IN] \~chinese Բ����Ϣ                        \~english Circle Point Info

    float           fR1;            ///< [IN] \~chinese ����뾶������ͼ������λ��[0, 1.0]���뾶��Բ�ĵ�λ���йأ��豣֤������Բ����ʾ��Χ֮�ڣ����򱨴�  \~english Windth Radius, Range[0, 1.0]
    float           fR2;            ///< [IN] \~chinese ����뾶������ͼ������λ��[0, 1.0]���뾶��Բ�ĵ�λ���йأ��豣֤������Բ����ʾ��Χ֮�ڣ����򱨴�  \~english Height Radius, Range[0, 1.0]

    MVCC_COLORF     stColor;        ///< [IN] \~chinese ��������ɫ��Ϣ                  \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///< [IN] \~chinese �����߿�ȣ����ֻ����1��2      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<      \~chinese Ԥ��                            \~english Reserved

}MVCC_CIRCLE_INFO;

/// \~chinese ������������Ϣ    \~english Linear Auxiliary Line Info
typedef struct _MVCC_LINES_INFO
{
    MVCC_POINTF     stStartPoint;   ///< [IN] \~chinese ���������ߵ���ʼ������          \~english The Start Point of Auxiliary Line
    MVCC_POINTF     stEndPoint;     ///< [IN] \~chinese ���������ߵ��յ�����            \~english The End Point of Auxiliary Line
    MVCC_COLORF     stColor;        ///< [IN] \~chinese ��������ɫ��Ϣ                  \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///< [IN] \~chinese �����߿�ȣ����ֻ����1��2      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<  \~chinese Ԥ��                            \~english Reserved

}MVCC_LINES_INFO;

///< \~chinese ��ʱ�ع�ʱ��ཫԴͼ���ֵĸ��� \~english The maximum number of source image to be split in time-division exposure
#define MV_MAX_SPLIT_NUM                  8

/// \~chinese ͼ���ع��ķ�ʽ        \~english Image reconstruction method
typedef enum _MV_IMAGE_RECONSTRUCTION_METHOD_
{
    MV_SPLIT_BY_LINE                   = 1, ///< \~chinese Դͼ���в�ֳɶ���ͼ��         \~english Source image split into multiple images by line

}MV_IMAGE_RECONSTRUCTION_METHOD;

/// \~chinese ͼ���ع����ͼ���б�      \~english List of images after image reconstruction
typedef struct _MV_OUTPUT_IMAGE_INFO_
{
    unsigned int                    nWidth;        ///< [OUT]       \~chinese Դͼ���              \~english Source Image Width
    unsigned int                    nHeight;       ///< [OUT]       \~chinese Դͼ���              \~english Source Image Height
    enum MvGvspPixelType            enPixelType;   ///< [OUT]       \~chinese ���ظ�ʽ              \~english Pixel format

    unsigned char*                  pBuf;          ///< [IN][OUT]   \~chinese ������ݻ���          \~english Output data buffer
    unsigned int                    nBufLen;       ///< [OUT]       \~chinese ������ݳ���          \~english Output data length
    unsigned int                    nBufSize;      ///< [IN]        \~chinese �ṩ�������������С  \~english Provided output buffer size

    unsigned int                    nRes[8];       ///<             \~chinese Ԥ��                  \~english Reserved
}MV_OUTPUT_IMAGE_INFO;

/// \~chinese �ع�ͼ�������Ϣ      \~english Restructure image parameters
typedef struct _MV_RECONSTRUCT_IMAGE_PARAM_
{
    unsigned int                    nWidth;                             ///< [IN]  \~chinese Դͼ���          \~english Source Image Width
    unsigned int                    nHeight;                            ///< [IN]  \~chinese Դͼ���          \~english Source Image Height
    enum MvGvspPixelType            enPixelType;                        ///< [IN]  \~chinese ���ظ�ʽ          \~english Pixel format

    unsigned char*                  pSrcData;                           ///< [IN]  \~chinese �������ݻ���      \~english Input data buffer
    unsigned int                    nSrcDataLen;                        ///< [IN]  \~chinese �������ݳ���      \~english Input data length

    unsigned int                    nExposureNum;                       ///< [IN]  \~chinese �ع����(1-8]     \~english Exposure number
    MV_IMAGE_RECONSTRUCTION_METHOD  enReconstructMethod;                ///< [IN]  \~chinese ͼ���ع���ʽ      \~english Image restructuring method

    MV_OUTPUT_IMAGE_INFO            stDstBufList[MV_MAX_SPLIT_NUM];     ///< [OUT] \~chinese ������ݻ�����Ϣ  \~english Output data info

    unsigned int                    nRes[4];
}MV_RECONSTRUCT_IMAGE_PARAM;

/// \~chinese ������Ϣ      \~english Serial Port Info
typedef struct _MV_CAML_SERIAL_PORT_
{
    unsigned char       chSerialPort[INFO_MAX_BUFFER_SIZE];             ///< [OUT] \~chinese ���ں�                 \~english Serial Port

    unsigned int        nRes[4];                                       ///<\~chinese Ԥ��                           \~english Reserved
}MV_CAML_SERIAL_PORT;

///< \~chinese ���֧�ֵĴ�������  \~english The maximum number of serial port supported
#define MV_MAX_SERIAL_PORT_NUM     64

typedef struct _MV_CAML_SERIAL_PORT_LIST_
{
    unsigned int                  nSerialPortNum;                        //< [OUT] \~chinese ��������                  \~english Serial Port Num
    MV_CAML_SERIAL_PORT           stSerialPort[MV_MAX_SERIAL_PORT_NUM];  //< [IN][OUT] \~chinese ������Ϣ               \~english Serial Port Information

    unsigned int                  nRes[4];                               ///<\~chinese Ԥ��                             \~english Reserved
}MV_CAML_SERIAL_PORT_LIST;

#endif /* _MV_CAMERA_PARAMS_H_ */
