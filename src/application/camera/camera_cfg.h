#ifndef __CAMERA_CFG_H__
#define __CAMERA_CFG_H__

#include "camera_control.h"
#include "hb_mipi_api.h"
#include "hb_vin_api.h"

#include "camera_x3m_cfg_1920x1080.h"
//#include "camera_positec_cfg.h"

int GetBoardType(void);
int GetVideoDevPath(CAM_TYPE_E eCamType, char *pcCamDirection, int *piCamDevId, PIPELINE_TYPE_E *pePipeType);
int GetCamI2cSlaveAddr(CAM_TYPE_E eCamType, char *pcCamDirection, unsigned int *puiSlaveAddr);
int GetCamI2cDevId(CAM_TYPE_E eCamType, char *pcCamDirection, int *piI2cDevId);

VIN_DEV_ATTR_S *GetIrs2381cVinDevAttr(void);
VIN_PIPE_ATTR_S *GetIrs2381cVinPipeAttr(void);
MIPI_SENSOR_INFO_S *GetIrs2381cSensorInfo(void);
MIPI_ATTR_S *GetIrs2381cMipiAttr(void);
VIN_DEV_ATTR_S *GetGc2053VinDevAttr(void);
VIN_PIPE_ATTR_S *GetGc2053VinPipeAttr(void);
MIPI_SENSOR_INFO_S *GetGc2053SensorInfo(void);
MIPI_ATTR_S *GetGc2053MipiAttr(void);


#endif /* __CAMERA_CFG_H__ */
