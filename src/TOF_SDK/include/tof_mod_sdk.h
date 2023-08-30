#ifndef __TOF_MODULE_SDK_H__
#define __TOF_MODULE_SDK_H__

#include "tof_sdk_typedef.h"

#ifdef WIN32
    #ifdef TOF_MODULE_SDK_EXPORT
        #define TOFMDLL __declspec(dllexport)
    #else
        #define TOFMDLL __declspec(dllimport)
    #endif
#else
    #define TOFMDLL __attribute__((visibility("default")))
#endif


typedef struct tagTofModuleInitParam
{
	SCHAR szDepthCalcCfgFileDir[200];//深度计算所需配置文件的目录，如home/user/temp
	UINT8 nLogLevel;//日志打印级别

}TofModuleInitParam;

typedef struct tagTofModuleCapability
{
	TOF_MODE tofMode;
	UINT32 tofResWidth;//输出的数据的宽
	UINT32 tofResHeight;//输出的数据的高

	//TOF HDRZ
	SBOOL bTofHDRZSupported;

	//TOF RemoveINS
	SBOOL bTofRemoveINSSupported;

	//TOF 多路径矫正
	SBOOL bTofMpiCorrectSupported;

	//TOF 多路径矫正结果和传统算法的融合
	SBOOL bTofMpiFuseSupported;

	//TOF Filter
	UINT32 supportedTOFFilter; //TOF_FILTER的组合

	//TOF滤波等级
	SBOOL bTofFilterLevelSupported;
	UINT8 nTofFilterLevelMin; //最小TOF滤波等级（宽松）
	UINT8 nTofFilterLevelMax; //最大TOF滤波等级（严格）

}TofModuleCapability;


typedef struct tagTofModuleCaps
{	
	UINT32 capCnt; //实际使用的能力个数，不超过TOF_MAX_CAPS_CNT
	TofModuleCapability cap[TOF_MAX_CAPS_CNT];//每一种模式下的能力

}TofModuleCaps;

typedef enum tagMODULE_NAME
{
	MODULE_NAME_FIRST = 0,

	MODULE_NAME_MD101D = MODULE_NAME_FIRST,
	MODULE_NAME_MTP004, 
	MODULE_NAME_MTP004C,
	MODULE_NAME_MTP006,
	MODULE_NAME_MTP007,
	MODULE_NAME_MTP008,
	MODULE_NAME_MTP009,
	MODULE_NAME_MTP009A,
	MODULE_NAME_MTP012,
	MODULE_NAME_MTT010,
	MODULE_NAME_MTT011,
	MODULE_NAME_MTT013,
	MODULE_NAME_MTT014,
	MODULE_NAME_MTT015,
	MODULE_NAME_MTT015A,
	MODULE_NAME_MTT016,
	MODULE_NAME_MTT020,
	MODULE_NAME_YMTT002,
	MODULE_NAME_YMTT003,
	MODULE_NAME_MTP013,



	MODULE_NAME_LAST,//新增模组型号放在该枚举值之前（为了兼容，需按先后顺序添加，不得改变原有枚举取值）
}MODULE_NAME;


typedef void* HTOFM;

typedef struct tagTofModuleHal
{
	SBOOL(*Init)(void* user_data);
	SBOOL(*Deinit)(void* user_data);
	
	/**********写入一个值到寄存器**********/
	//@    slave_addr:  从机地址;
	//@    regAddr:      寄存器地址;
	//@    value:           要写入寄存器的值;
	//@    返回值:       成功或失败;
	SBOOL(*WriteReg16)(const UINT8 slave_addr, const UINT16 regAddr, const UINT16 value, void*user_data);//根据模组实际情况选择是否实现

	/**********读取一个寄存器的值**********/
	//@    slave_addr:  从机地址;
	//@    regAddr:      寄存器地址;
	//@    value:           读取到的值;
	//@    返回值:       成功或失败;
	SBOOL(*ReadReg16)(const UINT8 slave_addr, const UINT16 regAddr, UINT16 *value, void*user_data);//根据模组实际情况选择是否实现

	/**********读取多个寄存器的值**********/
	//@    slave_addr:  从机地址;
	//@    startAddr:    要读取的起始地址;
	//@    addrCnt:      要读取的地址个数;
	//@    pBufOut:     存放读取到的数据;
	//@    返回值:      读取到的数据长度;
	UINT32(*ReadRegBulk)(const UINT8 slave_addr, const UINT32 startAddr, const UINT32 addrCnt, void *pBufOut, void*user_data);//根据模组实际情况选择是否实现


}TofModuleHal;


typedef struct tagTofModDepthData
{
	UINT32 frameWidth;//输出的数据的宽
	UINT32 frameHeight;//输出的数据的高

	//
	FLOAT32* pDepthData;//射线距离（滤波前）
	FLOAT32* pDepthDataFilter;//射线距离（滤波后）
	//
	PointData* pPointData;//点云数据
	PointData* pPointDataUnfilter;//点云数据（滤波前）
	//
	UINT8* pGrayData;//灰度数据
	UINT8* pConfidence;//置信度数据
	UINT8* pIntensity;//环境光数据
	//
	UINT8* pMaxValidPixelFlag;//最大有效像素标记

	TofExpouseCurrentItems autoExp;//计算出的自动曝光值（当需要自动曝光效果时，需要将该值设置到模组中）

	FLOAT32 temperature;//温度

	UINT32 nPixelOffset;//相对于原始TOF数据分辨率下的数据,输出的数据跳过的像素个数

	SBOOL bHighReflectDetected;//是否检测到高反物体

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void*  pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}TofModDepthData;


typedef struct tagSomeCalibParam
{
	//
	TofModuleLensParameter struLensParamter;//TOF模组内参和畸变（V1.0版本）建议不要再用，因为不能适用于鱼眼模型）
	TofModuleLensParameterV20 struLensParamterV20;//TOF模组内参和畸变（V2.0版本）

	//

}SomeCalibParam;



#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************/
/*********************第一部分：SDK基本接口（公用）**************************/
/*****************************************************************************/

//初始化/反初始化SDK（其他任何接口的使用都必须介于这两个接口之间，只需要调用一次）
TOFMDLL TOFRET TOFM_Init(TofModuleInitParam* pInitParam);
TOFMDLL TOFRET TOFM_Uninit(void);

//获取SDK版本号（返回值为字符串型版本号）
TOFMDLL SCHAR* TOFM_GetSDKVersion(void);

//创建/释放句柄资源
//老接口（逐渐废弃，仅适用于MODULE_NAME枚举）
TOFMDLL HTOFM  TOFM_OpenDevice(const MODULE_NAME mod_name, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//新接口（兼容老接口，但是MODULE_NAME枚举不支持的模组型号一定要用新接口）
TOFMDLL HTOFM  TOFM_OpenDeviceV20(SCHAR* pModName, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//新接口（兼容老接口，但是MODULE_NAME枚举不支持的模组型号一定要用新接口，相对于V20接口，可以实现不同客户对同一模组的不同数据需求）
TOFMDLL HTOFM  TOFM_OpenDeviceV30(SCHAR* pModName, const MODULE_GUEST_ID guestID, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//新接口（无法兼容老接口）
TOFMDLL HTOFM  TOFM_OpenDeviceV40(TofModuleDescriptor* pModDesc, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
TOFMDLL TOFRET TOFM_CloseDevice(HTOFM hTofMod);

//绑定TOF 模式（关系到后续的标定数据解析、深度计算初始化）
//老接口（采用默认的配置文件）
TOFMDLL TOFRET TOFM_SetTofMode(HTOFM hTofMod, const TOF_MODE tofMode);
//新接口（可以重新指定配置文件）, pModCfgFile为配置文件完整路径并且可以为NULL值（NULL时表示采用默认配置文件）
TOFMDLL TOFRET TOFM_SetTofModeV20(HTOFM hTofMod, const TOF_MODE tofMode, SCHAR* pModCfgFile);

//绑定一些回调函数（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_SetExterntionHooks(HTOFM hTofMod, ExterntionHooks* pHooks);


/*****************************************************************************/
/*********************第二部分：该部分为模组硬件相关操作（硬件交互）**********/
/*****************************************************************************/

//设置模组曝光
TOFMDLL TOFRET TOFM_SetTofExpTime(HTOFM hTofMod, TofExpouseCurrentItems* pExp);
//获取模组温度
TOFMDLL TOFRET TOFM_GetTemperature(HTOFM hTofMod, FLOAT32* pTemperature);


/*****************************************************************************/
/*********************第三部分：该部分为深度计算相关算法（软件算法）************/
/****该部分其他接口借用时必须介于TOFM_InitDepthCal、TOFM_UnInitDepthCal之间****/
/*****************************************************************************/

//深度计算模块
TOFMDLL TOFRET TOFM_InitDepthCal(HTOFM hTofMod, UINT8* pCalibData, const UINT32 nCalibDataLen);//(比较耗时)
TOFMDLL TOFRET TOFM_UnInitDepthCal(HTOFM hTofMod);

TOFMDLL TOFRET TOFM_DetectMultiDevInterference(HTOFM hTofMod, TofRawData* pRawData, MULTI_DEV_INTERFERENCE* pInterference);
TOFMDLL TOFRET TOFM_DoDepthCal(HTOFM hTofMod, TofRawData* pRawData, TofModDepthData* pDataOut);

//获取/设置滤波等级，取值范围:[TofModuleCapability的nTofFilterLevelMin, nTofFilterLevelMax]
TOFMDLL TOFRET TOFM_GetTofFilterLevel(HTOFM hTofMod, UINT32* pnLevel);
TOFMDLL TOFRET TOFM_SetTofFilterLevel(HTOFM hTofMod, const UINT32 nLevel);
//启用/禁用某种滤波算法
TOFMDLL TOFRET TOFM_GetTofFilter(HTOFM hTofMod, const TOF_FILTER type, SBOOL* pbEnable);
TOFMDLL TOFRET TOFM_SetTofFilter(HTOFM hTofMod, const TOF_FILTER type, const SBOOL bEnable);

//启用/禁用HDRZ算法
TOFMDLL TOFRET TOFM_SetTofHDRZ(HTOFM hTofMod, const SBOOL bEnable);

//启用/禁用RemoveINS算法
TOFMDLL TOFRET TOFM_SetTofRemoveINS(HTOFM hTofMod, const SBOOL bEnable);

//启用/禁用多路径矫正
TOFMDLL TOFRET TOFM_SetTofMpiCorrect(HTOFM hTofMod, const SBOOL bEnable);

//启用/禁用多路径矫正结果和传统算法的融合
TOFMDLL TOFRET TOFM_SetTofMpiFuse(HTOFM hTofMod, const SBOOL bEnable);

//读取标定数据内部分标定参数（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_GetSomeCalibParam(HTOFM hTofMod, SomeCalibParam* pParamOut);

//调整深度计算的ROI（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_GetDepthCalRoi(HTOFM hTofMod, DepthCalRoi* pRoi);
TOFMDLL TOFRET TOFM_SetDepthCalRoi(HTOFM hTofMod, DepthCalRoi* pRoi);

//获取曝光上下限（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_GetTofExpTimeRange(HTOFM hTofMod, TofExpouseRangeItems* pRange);




#ifdef __cplusplus
}
#endif

#endif

