#ifdef WIN32

	#include <Windows.h>
	#include <direct.h>
	#include <psapi.h>
	#include <io.h>

	#define R_OK 4
	#define W_OK 2
	#define X_OK 1
	#define F_OK 0

#elif defined LINUX 

	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/sysinfo.h>

#endif
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <vector>
#include <chrono>
#include "tof_mod_sdk.h"

#include "tof_depth_process.h"
#include "tof_i2c.h"
#include "camera_control.h"
#include "camera_cfg.h"

//
#define SAFE_DELETE(p) if(p){delete p; p=NULL;}
#define SAFE_DELETE_ARRY(p) if(p){delete [] p; p=NULL;}
#define SAFE_FREE(p) if(p){free(p); p=NULL;}
#define SAFE_CLOSE_FP(fp) if(fp){fclose(fp); fp=NULL;}
#define LINUX


#ifdef WIN32
static int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

static unsigned long long Utils_GetTickCount(void)
{
	unsigned long long tick = 0;

#ifdef WIN32
	//tick = GetTickCount();//实际精度只有15ms左右; 返回的是一个32位的无符号整数，Windows连续运行49.710天后，它将再次从零开始计时; 
	//tick = GetTickCount64();//返回一个64位的无符号整数。Windows连续运行5.8亿年后，其计时才会归零; 
	//tick = clock();//该程序从启动到函数调用占用CPU的时间, 是C/C++中的计时函数

	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
		
	auto timePoint = std::chrono::steady_clock::now(); // std::chrono::time_point
	tick = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count();

#elif defined LINUX
	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);
	
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
	tick = (tv.tv_sec * 1000 + tv.tv_nsec/1000000);
	
#else
	printf("unknown platform in getting tick cnt, error!\n");
#endif // WIN32

	return tick;
}


static long Utils_GetFileLen(const char * filename)
{
	if (NULL == filename)
	{
		printf("NULL == filename\n");
		return 0;
	}

	FILE * fd = fopen(filename, "rb");
	if (NULL == fd)
	{
		printf("open file (%s) failed, errno=%d(%s).\n", filename, errno, strerror(errno));
		return 0;
	}

	fseek(fd, 0L, SEEK_END); /* 定位到文件末尾 */
	const long len = ftell(fd);
	fclose(fd);

	return len;

}

static void Utils_SaveBufToFile(void* pData, const unsigned int nDataLen, const char* pFile, const bool bAppend)
{
	if ((NULL == pData) || (0 >= nDataLen) || (NULL == pFile))
	{
		return;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return;
	}

	fwrite(pData, 1, nDataLen, fp);
	fclose(fp);
}

template <class T>
T Utils_FindMaxValue(T* pData, const int nCnt)
{
	T max = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (max < pData[i])
		{
			max = pData[i];
		}
	}

	return max;
}

template <class T>
T Utils_FindMinValue(T* pData, const int nCnt)
{
	T min = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (min > pData[i])
		{
			min = pData[i];
		}
	}

	return min;
}

float CalCenterPointDataZAvg(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if (NULL == pPointData)
	{
		return 0;
	}

	const int start_h = (10<height) ? ((height / 2) - 5) : 0;
	const int end_h = (10<height) ? ((height / 2) + 5) : (height);
	const int start_w = (10<width) ? ((width / 2) - 5) : 0;
	const int end_w = (10<width) ? ((width / 2) + 5) : (width);


	float sum = 0.0;
	int cnt = 0;
	for (int h = start_h; h < end_h; h++)
	{
		PointData *pTmp = pPointData + h*width;
		for (int w = start_w; w < end_w; w++)
		{
			if (0.00001 < pTmp[w].z)
			{
				sum += pTmp[w].z;
				cnt++;
			}
		}
	}

	return ((0 < cnt) ? (sum / cnt) : 0);
}

static const char* TofMode2Str(const TOF_MODE mode)
{
	const char* pStr = "Unknown";

	switch (mode)
	{
	case TOF_MODE_STERO_5FPS: pStr = "STERO_5FPS"; break;
	case TOF_MODE_STERO_10FPS: pStr = "STERO_10FPS"; break;
	case TOF_MODE_STERO_15FPS: pStr = "STERO_15FPS"; break;
	case TOF_MODE_STERO_30FPS: pStr = "STERO_30FPS"; break;
	case TOF_MODE_STERO_45FPS: pStr = "STERO_45FPS"; break;
	case TOF_MODE_STERO_60FPS: pStr = "STERO_60FPS"; break;

	case TOF_MODE_MONO_5FPS: pStr = "MONO_5FPS"; break;
	case TOF_MODE_MONO_10FPS: pStr = "MONO_10FPS"; break;
	case TOF_MODE_MONO_15FPS: pStr = "MONO_15FPS"; break;
	case TOF_MODE_MONO_30FPS: pStr = "MONO_30FPS"; break;
	case TOF_MODE_MONO_45FPS: pStr = "MONO_45FPS"; break;
	case TOF_MODE_MONO_60FPS: pStr = "MONO_60FPS"; break;

	case TOF_MODE_HDRZ_5FPS: pStr = "HDRZ_5FPS"; break;
	case TOF_MODE_HDRZ_10FPS: pStr = "HDRZ_10FPS"; break;
	case TOF_MODE_HDRZ_15FPS: pStr = "HDRZ_15FPS"; break;
	case TOF_MODE_HDRZ_30FPS: pStr = "HDRZ_30FPS"; break;
	case TOF_MODE_HDRZ_45FPS: pStr = "HDRZ_45FPS"; break;
	case TOF_MODE_HDRZ_60FPS: pStr = "HDRZ_60FPS"; break;

	case TOF_MODE_5FPS: pStr = "5FPS"; break;
	case TOF_MODE_10FPS: pStr = "10FPS"; break;
	case TOF_MODE_20FPS: pStr = "20FPS"; break;
	case TOF_MODE_30FPS: pStr = "30FPS"; break;
	case TOF_MODE_45FPS: pStr = "45FPS"; break;
	case TOF_MODE_60FPS: pStr = "60FPS"; break;

	case TOF_MODE_ADI_1M5: pStr = "ADI_1M5"; break;
	case TOF_MODE_ADI_5M: pStr = "ADI_5M"; break;

	default: break;
	}

	return pStr;
}

static bool SaveDepthText(float* pDepthData, const UINT32 width, const UINT32 height, char* pTxtFile, const bool bWH)
{
	if ((NULL == pDepthData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	if (bWH)//1：W列、H行排列
	{
		UINT32 nPos = 0;
		for (UINT32 h = 0; h < height; h++)
		{
			for (UINT32 w = 0; w < (width - 1); w++)
			{
				fprintf(fp, "%0.6f,", pDepthData[nPos]);
				nPos++;
			}
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
			nPos++;
		}
	}
	else//2：1行、W*H行排列
	{
		const UINT32 nCnt = width *height;
		for (UINT32 nPos = 0; nPos < nCnt; nPos++)
		{
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
		}
	}

	fclose(fp);
	return true;
}
static bool SavePointDataXYZText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;%0.6f;%0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z);
	}

	fclose(fp);
	return true;
}
static bool SavePointDataZWHText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	UINT32 nPos = 0;
	for (UINT32 h = 0; h < height; h++)
	{
		for (UINT32 w = 0; w < width; w++)
		{
			fprintf(fp, "%0.6f", pPointData[nPos].z);
			nPos++;
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	return true;
}

static bool SaveFloatPointDataText(FLOAT32 *pfPoint, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pfPoint) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;\n", pfPoint[nPos]);
	}

	fclose(fp);
	return true;
}

static bool SaveU16PointDataText(UINT16 *pu16Point, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pu16Point) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%u;\n", pu16Point[nPos]);
	}

	fclose(fp);
	return true;
}



class CBuf
{
public:
	CBuf(const unsigned int nBufLen = 128/*字节数*/)
	{
		m_nBufLen = (0 < nBufLen) ? nBufLen : 128;//防止参数错误
		m_pBuf = new unsigned char[m_nBufLen];
		m_nDataLen = 0;

		memset(m_pBuf, 0, m_nBufLen);
	}
	virtual ~CBuf()
	{
		if (m_pBuf)
		{
			delete[] m_pBuf;
			m_pBuf = NULL;
		}
	}

public:
	unsigned char* GetBuf(void)
	{
		return m_pBuf;
	}
	unsigned int GetBufLen(void)
	{
		return m_nBufLen;
	}
	unsigned int GetDataLen(void)
	{
		return m_nDataLen;
	}
	unsigned int SetDataLen(const unsigned int nDataLen)
	{
		m_nDataLen = ((nDataLen > m_nBufLen) ? m_nBufLen : nDataLen);//不允许超过缓冲区长度

		return m_nDataLen;
	}
	void ReSize(const unsigned int nBufLen)
	{
		if (m_pBuf)
		{
			delete[] m_pBuf;
			m_pBuf = NULL;
		}

		m_nBufLen = (0 < nBufLen) ? nBufLen : 128;//防止参数错误
		m_pBuf = new unsigned char[m_nBufLen];
		m_nDataLen = 0;

		memset(m_pBuf, 0, m_nBufLen);
	}


private:
	unsigned char* m_pBuf;//指向一块缓冲区
	unsigned int m_nBufLen;//m_pBuf长度
	unsigned int m_nDataLen;//m_pBuf内数据长度

};


class CTofModuleHalUserData
{
public:
	CTofModuleHalUserData(const char* pCalibFile)
	{
		m_strCalibFile = pCalibFile;
		if (NULL == (m_fpCalibFile = fopen(pCalibFile, "rb")))
		{
			printf("open calib file %s failed.\n:", pCalibFile);
		}
	}
	virtual ~CTofModuleHalUserData()
	{
		m_strCalibFile = "";
		if (m_fpCalibFile)
		{
			fclose(m_fpCalibFile);
			m_fpCalibFile = NULL;
		}
	}

public:
	FILE* GetCalibFile(void)
	{
		return m_fpCalibFile;
	}

private:
	std::string m_strCalibFile;
	FILE* m_fpCalibFile;

};


static SBOOL TofModuleHal_Init(void* user_data)
{
	//anything you can do, by youself

	CTofModuleHalUserData*pInput = (CTofModuleHalUserData*)user_data;


	return true;
}
static SBOOL TofModuleHal_Deinit(void* user_data)
{
	//anything you can do, by youself

	CTofModuleHalUserData*pInput = (CTofModuleHalUserData*)user_data;


	return true;
}

static SBOOL TofModuleHal_WriteReg16(const UINT8 slave_addr, const UINT16 regAddr, const UINT16 value, void*user_data)
{
	//generally , it is used to write a value to a sensor register
	I2C_CB_S *pstI2cCb = (I2C_CB_S*)user_data;

	I2cWrite(pstI2cCb, regAddr, value, I2C_FMT_A16D16);

	return true;
}

static SBOOL TofModuleHal_ReadReg16(const UINT8 slave_addr, const UINT16 regAddr, UINT16 *value, void*user_data)
{
	//generally , it is used to read  a value from a sensor register
	I2C_CB_S *pstI2cCb = (I2C_CB_S*)user_data;

	I2cRead(pstI2cCb, regAddr, value, I2C_FMT_A16D16);

	return true;
}

static UINT32 TofModuleHal_ReadRegBulk(const UINT8 slave_addr, const UINT32 startAddr, const UINT32 addrCnt, void *pBufOut, void*user_data)
{
	//generally , it is used to read calib data from eeprom
	//but, in some modules(without eeprom, but with spi flash), it is not used.

	CTofModuleHalUserData*pInput = (CTofModuleHalUserData*)user_data;
	if (pInput)
	{
		return ((UINT32)fread(pBufOut, 1, addrCnt, pInput->GetCalibFile()));
	}
	return 0;
}

static void ExterntionHooks_RecvTofExpTime(TofExpouseCurrentItems* pExp, void*user_data)
{
	if (1 == pExp->nIndex)
	{
		printf("RecvTofExpTime: time:%llu, exp:%d.\n", Utils_GetTickCount(), pExp->uParam.g1.exp);
	}
	else if (2 == pExp->nIndex)
	{
		printf("RecvTofExpTime: time:%llu, exp_AEF:%d, exp_FEF:%d.\n", Utils_GetTickCount(), pExp->uParam.g2.exp_AEF, pExp->uParam.g2.exp_FEF);
	}
	else
	{
		printf("ExterntionHooks RecvTofExpTime: time:%u, not supported.\n", pExp->nIndex);
		return;
	}

	HTOFM hDev = (HTOFM)user_data;
	const TOFRET retVal = TOFM_SetTofExpTime(hDev, pExp);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("ExterntionHooks: TOFM_SetTofExpTime failed, retVal=0x%08x.\n", retVal);
	}
}


static SBOOL ReadFile(std::string& strFile, CBuf& buf)
{
	const long file_len = Utils_GetFileLen(strFile.c_str());
	if (0 >= file_len)
	{
		printf("read file(%s), failed.......\n", strFile.c_str());
		return false;
	}

	if (file_len > (const long)buf.GetBufLen())
	{
		buf.ReSize(file_len);
	}

	FILE* fp = fopen(strFile.c_str(), "rb");
	const size_t read_len = fread(buf.GetBuf(), 1, file_len, fp);
	fclose(fp);

	buf.SetDataLen((const unsigned int)read_len);

	printf("read file(%s), ok, file_len=%ld, read_len=%lu.......\n", strFile.c_str(), file_len, read_len);

	return true;
}

static void CaptureTofFrame(const std::string& strDir, const std::string& strTofName, const unsigned int nCaptureIndex, TOF_DEPTH_DATA_INFO_S *tofFrameData)
{
	const unsigned int nPixelCnt = tofFrameData->frameWidth * tofFrameData->frameHeight;
	char szFile[512] = { 0 };

	//
	if (NULL != tofFrameData->pDepthData)
	{
		sprintf(szFile, "%s/%s-%u-DepthData.dat", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pDepthData, nPixelCnt  * sizeof(tofFrameData->pDepthData[0]), szFile, false);

		sprintf(szFile, "%s/%s-%u-DepthData.txt", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pDepthData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}

	//
	if (NULL != tofFrameData->pfPointData)
	{
		sprintf(szFile, "%s/%s-%u-PointData.dat", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pfPointData, nPixelCnt  * sizeof(tofFrameData->pfPointData[0]), szFile, false);

		sprintf(szFile, "%s/%s-%u-PointData.txt", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		SavePointDataXYZText(tofFrameData->pfPointData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	}

	//
	if (NULL != tofFrameData->pu8GrayData)
	{
		sprintf(szFile, "%s/%u-Gray.u8", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pu8GrayData, nPixelCnt * sizeof(tofFrameData->pu8GrayData[0]), szFile, false);
	}
	//
	/*
	if ((NULL != tofFrameData->pExtData) && (0 < tofFrameData->nExtDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "extdata");
		Utils_SaveBufToFile(tofFrameData->pExtData, tofFrameData->nExtDataLen, szFile, false);
	}
	*/
}

typedef struct tagSunnyConciseDepthData
{
	unsigned short* u16Depth;
	unsigned char* u8Gray;
}SunnyConciseDepthData;

typedef struct tagSunnyFullDepthData
{
	unsigned short* u16Depth;
	unsigned char* u8Gray;
	unsigned char* u8Intensity;
	unsigned short* u16Confidence;
}SunnyFullDepthData;

void HandleDepthData(const UINT32 threadIndex, UINT32 frameIndex, std::string& strSaveDir, std::string strTofName, TOF_DEPTH_DATA_INFO_S* tofFrameData)
{
	//todo anthing by youself. for example, save to file:

	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	const float fDepthZAvg = CalCenterPointDataZAvg(tofFrameData->pfPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	printf("[%u], one TOF frame,time=%llu, center depthZ = %0.3f m.\n", threadIndex, Utils_GetTickCount(), fDepthZAvg);

	CaptureTofFrame(strSaveDir, strTofName, frameIndex, tofFrameData);
}

#ifdef RGBD
static bool SaveColorPointDataXYZText(TofRgbdPointCloudColor *pPointData, const unsigned int width, const unsigned int height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const unsigned int nCnt = width *height;
	for (unsigned int nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "v %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z, pPointData[nPos].r, pPointData[nPos].g, pPointData[nPos].b);
	}

	fclose(fp);
	return true;
}

static bool SavePixelCoordText(TofRgbdPixelCoord *pPixelCoord, const unsigned int width, const unsigned int height, char* pTxtFile)
{
	if ((NULL == pPixelCoord) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const unsigned int nCnt = width *height;
	for (unsigned int nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%u %u\n", pPixelCoord[nPos].x, pPixelCoord[nPos].y);
	}

	fclose(fp);
	return true;
}

unsigned int ConvertPixelCoordMap2Rgb(unsigned char* pRgb, unsigned int nRgbWidth, unsigned int nRgbHeight, TofRgbdPixelCoord* pCoord, unsigned int nCoordWidth, unsigned int nCoordHeight, unsigned char* pOutRgb)
{
	for (int i = 0; i < nCoordWidth *nCoordHeight; i++)
	{
		unsigned int rgbpixle = pCoord[i].x + pCoord[i].y * nRgbWidth;
		if (0 != rgbpixle)
		{
			memcpy(pOutRgb + 3 * i, pRgb + 3 * rgbpixle, 3);//目前只考虑3通道rgb的情况
		}
	}

	return (nCoordWidth * nCoordHeight * 3);
}

static void CaptureTofRgbdOutputData(const std::string& strDir, const unsigned int nCaptureIndex, TofRgbdInputData* pDataIn, TofRgbdOutputData *rgbdData)
{
	char szFile[512] = { 0 };

	//
	if (1)
	{
		TofRgbdImage_PointCloud& tmp = rgbdData->pointCloud2Rgb;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-pointCloud2Rgb.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);

			sprintf(szFile, "%s/%u-pointCloud2Rgb.txt", strDir.c_str(), nCaptureIndex);
			SavePointDataXYZText((PointData*)tmp.pData, tmp.nWidth, tmp.nHeight, szFile);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_U8& tmp = rgbdData->gray2Rgb;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-gray2Rgb.u8", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_Void& tmp = rgbdData->rgb2Tof;
		if ((NULL != tmp.pData) && (0 < tmp.nDataLen))
		{
			sprintf(szFile, "%s/%u-rgb2Tof.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, tmp.nDataLen, szFile, false);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_PointCloudColor& tmp = rgbdData->colorPointCloud;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-colorPointCloud.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);

			sprintf(szFile, "%s/%u-colorPointCloud.obj", strDir.c_str(), nCaptureIndex);
			SaveColorPointDataXYZText(tmp.pData, tmp.nWidth, tmp.nHeight, szFile);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_PixelCoord& tmp = rgbdData->rgb2TofPixelCoord;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-rgb2TofPixelCoord.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);

			sprintf(szFile, "%s/%u-rgb2TofPixelCoord.txt", strDir.c_str(), nCaptureIndex);
			SavePixelCoordText(tmp.pData, tmp.nWidth, tmp.nHeight, szFile);


			CBuf bufRgb(tmp.nWidth * tmp.nHeight * 3);//目前只考虑3通道rgb的情况
			const unsigned int nRetLen = ConvertPixelCoordMap2Rgb(pDataIn->pRgb, RGB_WIDTH, RGB_HEIGHT, tmp.pData, tmp.nWidth, tmp.nHeight, bufRgb.GetBuf());
			sprintf(szFile, "%s/%u-rgb2TofPixelCoord-rgb.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(bufRgb.GetBuf(), nRetLen, szFile, false);

		}
	}

	if (NULL != rgbdData->depth2rgb.pData)
	{
		TofRgbdImage_Float& tmp = rgbdData->depth2rgb;
		const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

		printf("111 tmp.nWidth = %u, tmp.nHeight = %u\n", tmp.nWidth, tmp.nHeight);
		sprintf(szFile, "%s/%u-depth2rgb.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(rgbdData->depth2rgb.pData, nPixelCnt  * sizeof(rgbdData->depth2rgb.pData[0]), szFile, false);

		sprintf(szFile, "%s/%u-depth2rgb.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(rgbdData->depth2rgb.pData, tmp.nWidth, tmp.nHeight, szFile, false);
	}

	//
	if (1)
	{
		TofRgbdImage_Priv& tmp = rgbdData->privData;
		if ((NULL != tmp.pData) && (0 < tmp.nDataLen))
		{
			sprintf(szFile, "%s/%u-privData.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, tmp.nDataLen, szFile, false);
		}
	}
}

void HandleTofRgbdOutputData(unsigned int frameIndex, const std::string& strSaveDir, TofRgbdInputData* pDataIn, TofRgbdOutputData* rgbdData)
{
	//todo anthing by youself. for example, save to file:

	CaptureTofRgbdOutputData(strSaveDir, frameIndex, pDataIn, rgbdData);
}
#endif

static void AlsoCanGetOrSetSomeParam(HTOFM hTofMod, TofModuleCaps* pCaps)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	//TofExpouseCurrentItems struExpCurrent;
	//memset(&struExpCurrent, 0, sizeof(struExpCurrent));
	//struExpCurrent.nIndex = 1;
	////struExpCurrent.nIndex = 2;
	//if (1 == struExpCurrent.nIndex)
	//{
	//	struExpCurrent.uParam.g1.exp = 100;
	//}
	//else if (2 == struExpCurrent.nIndex)
	//{
	//	struExpCurrent.uParam.g2.exp_AEF = 200;
	//	struExpCurrent.uParam.g2.exp_FEF = 100;
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_SetTofExpTime(hTofMod, &struExpCurrent)))
	//{
	//	printf("TOFM_SetTofExpTime failed, retVal=0x%08x.\n", retVal);
	//}
	//TofExpouseItems struExp;
	//memset(&struExp, 0, sizeof(struExp));
	//if (TOFRET_SUCCESS != (retVal = TOFM_GetTofExpTime(hTofMod, &struExp)))
	//{
	//	printf("TOFM_GetTofExpTime failed, retVal=0x%08x.\n", retVal);
	//}
	//else
	//{
	//	if (1 == struExp.nIndex)
	//	{
	//		printf("TOFM_GetTofExpTime, exp=%d.\n", struExp.uParam.g1.exp);
	//	}
	//	else if (2 == struExp.nIndex)
	//	{
	//		printf("TOFM_GetTofExpTime, exp_AEF=%d, exp_FEF=%d.\n", struExp.uParam.g2.exp_AEF, struExp.uParam.g2.exp_FEF);
	//	}
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_SetTofFilter(hTofMod, const TOF_FILTER type, true)))
	//{
	//	printf("TOFM_SetTofFilter failed, retVal=0x%08x.\n", retVal);
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_GetTofFilter(hTofMod, const TOF_FILTER type, SBOOL* pbEnable)))
	//{
	//	printf("TOFM_GetTofFilter failed, retVal=0x%08x.\n", retVal);
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_SetTofHDRZ(hTofMod, false)))
	//{
	//	printf("TOFM_SetTofHDRZ failed, retVal=0x%08x.\n", retVal);
	//}
}

static void GetOrSetSomeParam(HTOFM hTofMod, TofModuleCapability* pCaps)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	if (pCaps->bTofHDRZSupported)
	{
		if (TOFRET_SUCCESS != (retVal = TOFM_SetTofHDRZ(hTofMod, true)))
		{
			printf("TOFM_SetTofHDRZ failed, retVal=0x%08x.\n", retVal);
		}
	}

	if (pCaps->bTofRemoveINSSupported)
	{
		if (TOFRET_SUCCESS != (retVal = TOFM_SetTofRemoveINS(hTofMod, true)))
		{
			printf("TOFM_SetTofRemoveINS failed, retVal=0x%08x.\n", retVal);
		}
	}

	for (UINT32 i = 0; i < 32; i++)
	{
		UINT32 type = (1 << i);
		if (0 != (pCaps->supportedTOFFilter & type))
		{
			if (TOFRET_SUCCESS != (retVal = TOFM_SetTofFilter(hTofMod, (const TOF_FILTER)type, true)))
			{
				printf("TOFM_SetTofFilter(0x%08x) failed, retVal=0x%08x.\n", (TOF_FILTER)type, retVal);
			}
		}
	}
}


static void PrintfSomeCalibParam(SomeCalibParam* pParamOut)
{
	if (NULL == pParamOut)  return;

	if (2 == pParamOut->struLensParamterV20.nIndex)
	{
		printf("struLensParamter-fishEye:...............................\n");
		printf(">>   fx = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.fx);
		printf(">>   fy = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.fy);
		printf(">>   cx = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.cx);
		printf(">>   cy = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.cy);
		printf(">>   k1 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k1);
		printf(">>   k2 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k2);
		printf(">>   k3 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k3);
		printf(">>   k4 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k4);
	}
}

static bool ReadTemperature(HTOFM hTofMod, FLOAT32* pTemperature)
{
	FLOAT32 temperature = 0;
	const TOFRET retVal = TOFM_GetTemperature(hTofMod, &temperature);//不是所有的模组都支持这个接口
	if (TOFRET_SUCCESS == retVal)
	{
		*pTemperature = temperature;
		return true;
	}
	else if (TOFRET_ERROR_NOT_SUPPORTED == retVal)
	{
		*pTemperature = 0;
		return true;
	}
	else
	{
		printf("TOFM_GetTemperature failed, retVal=0x%08x.\n", retVal);
		return false;
	}
}

static void Sunny_RecvTofExpTime(TofExpouseCurrentItems* pExp, void *user_data)
{
	DEPTH_HANDLE_CB_S *pstDepthHandleCb = (DEPTH_HANDLE_CB_S*)user_data;

	if (1 == pExp->nIndex)
	{
		pstDepthHandleCb->stTofSetExp.expTime = pExp->uParam.g1.exp;
		pstDepthHandleCb->stTofSetExp.eExpMode = SINGLE_FRAME_MODE;
	}
	else if (2 == pExp->nIndex)
	{
		pstDepthHandleCb->stTofSetExp.expTime_AEF = pExp->uParam.g2.exp_AEF;
		pstDepthHandleCb->stTofSetExp.expTime_FEF = pExp->uParam.g2.exp_FEF;
		pstDepthHandleCb->stTofSetExp.eExpMode = LONG_SHORT_FRAME_MODE;
		
		//printf("start tof-%s AEF[%d] FEF[%d]\n", pstCameraHandler->acDirection, pExp->uParam.g2.exp_AEF, pExp->uParam.g2.exp_FEF);
	}
	
	pstDepthHandleCb->stTofSetExp.flag = START_SETUP_EXP;
}


void *Sunny_SetTofEXP(void *para)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	TofExpouseCurrentItems stExpCurtItems = {0};
	DEPTH_HANDLE_CB_S *pstDepthHandleCb = (DEPTH_HANDLE_CB_S*)para;
	
	while(pstDepthHandleCb->isExpTrdRunning)
	{
		if(pstDepthHandleCb->stTofSetExp.flag == START_SETUP_EXP )
		{
			memset(&stExpCurtItems, 0, sizeof(stExpCurtItems));
		
			if (pstDepthHandleCb->stTofSetExp.eExpMode == LONG_SHORT_FRAME_MODE)
			{
				stExpCurtItems.nIndex = 2;
				stExpCurtItems.uParam.g2.exp_AEF = pstDepthHandleCb->stTofSetExp.expTime_AEF;
				stExpCurtItems.uParam.g2.exp_FEF = pstDepthHandleCb->stTofSetExp.expTime_FEF;
				//printf("Set tof AEF[%d] FEF[%d]\n", pstDepthHandleCb->stTofSetExp.expTime_AEF, 
				//	pstDepthHandleCb->stTofSetExp.expTime_FEF);
			}
			else if (pstDepthHandleCb->stTofSetExp.eExpMode == SINGLE_FRAME_MODE)
			{
				stExpCurtItems.nIndex = 1;
				stExpCurtItems.uParam.g1.exp = pstDepthHandleCb->stTofSetExp.expTime;
				//printf("end EXP[%d]\n", stSetExp.expTime);
			}

			TOFM_SetTofExpTime(pstDepthHandleCb->hTofMod, &stExpCurtItems);
			pstDepthHandleCb->stTofSetExp.flag = STOP_SETUP_EXP;
		}
		
		usleep(1000);		
	}

	return NULL;
}


void TofDepthSdkGloableInit()
{

	//0. 初始化.......
	TofModuleInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, SELECT_MODULE_CFG_FILE_PATH, sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	TOFM_Init(&struInitParam);//必须放在最开始

	printf("SDK Version: %s.\n", TOFM_GetSDKVersion());
}


void TofDepthSdkGloableUnInit()
{
	TOFM_Uninit();//必须与TOFM_Init配套使用
}


int TofDepthSdkInit(DEPTH_HANDLE_CB_S *pstDepthHandleCb, char *pcCalibDataPath, void *pHalUserData)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	TofModuleCaps stTofModuleCaps;
	CBuf calibData(512 * 1024);//不能少于128K,，一般不超过128K
	TofModuleCapability* pCaps = NULL; //表明指定模式下的能力

	if (!pstDepthHandleCb || !pcCalibDataPath || !pHalUserData)
	{
		printf("NULL ptr!\n");
		return -1;
	}

	//1. 打开模组.......
	const std::string strModule = (SELECT_MODULE_NAME);

	const TOF_MODE tofMode = TOF_MODE_HDRZ_10FPS;
	std::string strCalibFile = pcCalibDataPath;

	TofModuleHal struHal;// 传入参数
	memset(&struHal, 0, sizeof(struHal));
	struHal.Init 		= TofModuleHal_Init;
	struHal.Deinit 		= TofModuleHal_Deinit;
	struHal.WriteReg16 	= TofModuleHal_WriteReg16;
	struHal.ReadReg16 	= TofModuleHal_ReadReg16;
	struHal.ReadRegBulk = TofModuleHal_ReadRegBulk;

	HTOFM hTofMod = TOFM_OpenDeviceV30((char*)(strModule.c_str()), SELECT_MODULE_GUEST_ID, &struHal, pHalUserData, &stTofModuleCaps);
	if (NULL == hTofMod)
	{
		printf("TOFM_OpenDevice failed.\n");
		return -1;
	}

	for (UINT32 i = 0; i < stTofModuleCaps.capCnt; i++)
	{
		if (tofMode == stTofModuleCaps.cap[i].tofMode)
		{
			pCaps = &(stTofModuleCaps.cap[i]);
			break;
		}
	}

	// 可自行指定配置文件名，为空则使用默认配置文件
	const std::string strCfgFile = ("");	
	
	SCHAR* pModCfgFile = (("" == strCfgFile) ? NULL : (SCHAR*)(strCfgFile.c_str()));
	if (TOFRET_SUCCESS != (retVal = TOFM_SetTofModeV20(hTofMod, tofMode, pModCfgFile)))
	{
		printf("TOFM_SetTofMode(tofMode:0x%08x) failed, retVal=0x%08x.\n", tofMode, retVal);
		goto errExitCloseDev;
	}

	//2. 从设备读取标定数据，可以备份着
	{
		if (!ReadFile(strCalibFile, calibData))
		{
			printf("ReadCalibData failed.\n");
			remove(pcCalibDataPath);
			goto errExitCloseDev;
		}
	}

	//4. 这时候可以读取一些标定参数出来备份着
	SomeCalibParam struSomeCalibParam;
    if (TOFRET_SUCCESS == (TOFM_GetSomeCalibParam(hTofMod, &struSomeCalibParam)))
    {
        PrintfSomeCalibParam(&struSomeCalibParam);
    }

	//深度计算模块（该部分必须在TOFM_LoadCalibData之后，TOFM_UnLoadCalibData之前调用）
	//5. 初始化深度计算模块
	if (TOFRET_SUCCESS != (retVal = TOFM_InitDepthCal(hTofMod, calibData.GetBuf(), calibData.GetDataLen())))//(比较耗时)
	{
		printf("TOFM_InitDepthCal failed, retVal=0x%08x.\n", retVal);
		goto errExitCloseDev;
	}

	//7. 一般在开流之后，可按需获取/修改参数
	GetOrSetSomeParam(hTofMod, pCaps);////ae 、 滤波

	// 7.1 记录支持的滤波类型
	pstDepthHandleCb->uiSupportedTofFilter = pCaps->supportedTOFFilter;

	//8. 曝光处理设置
	ExterntionHooks stHooks;
	memset(&stHooks, 0x00, sizeof(ExterntionHooks));
	stHooks.pUserData = pstDepthHandleCb;
	stHooks.RecvTofExpTime = Sunny_RecvTofExpTime;
	TOFM_SetExterntionHooks(hTofMod, &stHooks);
	
	/* Create set tof exp thread */
	pstDepthHandleCb->stTofSetExp.flag = STOP_SETUP_EXP;
	pstDepthHandleCb->isExpTrdRunning = 1;
	pthread_create(&pstDepthHandleCb->stTofExpPid, NULL, Sunny_SetTofEXP, pstDepthHandleCb);

	pstDepthHandleCb->hTofMod = hTofMod;

	printf("TofDepthSdkInit successed\n");	
	return 0;

errExitUnInitDepthCal:
	//10. 回收深度计算模块
	retVal = TOFM_UnInitDepthCal(hTofMod);

errExitCloseDev:
	//12. 关闭模组.......
	retVal = TOFM_CloseDevice(hTofMod);

	printf("TofDepthSdkInit failed\n");
	return -1;
}


int TofDepthSdkUnInit(DEPTH_HANDLE_CB_S *pstDepthHandleCb)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	if (!pstDepthHandleCb)
	{
		printf("[%s] NULL ptr\n", __func__);
		return -1;
	}

	if (NULL != pstDepthHandleCb->hTofMod)
	{
		//10. 回收深度计算模块
		retVal = TOFM_UnInitDepthCal(pstDepthHandleCb->hTofMod);

		//12. 关闭模组.......
		retVal = TOFM_CloseDevice(pstDepthHandleCb->hTofMod);
	}

	pstDepthHandleCb->isExpTrdRunning = 0;
	pthread_join(pstDepthHandleCb->stTofExpPid, NULL);

	return 0;
}

int TofDepthProcess(void *pCamHandle, IMAGE_DATA_INFO_S *pstTofRawDataInfo, TOF_DEPTH_DATA_INFO_S *pstTofDepthDataInfo)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	int i;
	unsigned int uiPixelNum = 0;
	float fConfidence = 0;
	TofRawData stRawData;
	TofModDepthData stTofDepthData;
	camera_handle *pstTofHandle;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;

	memset(&stRawData, 0, sizeof(stRawData));
	memset(&stTofDepthData, 0, sizeof(stTofDepthData));

	stRawData.nRawLen = pstTofRawDataInfo->uiImageSize;
	stRawData.pRaw = pstTofRawDataInfo->pucImageData;

	if (!pCamHandle || !pstTofRawDataInfo || !pstTofDepthDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstTofHandle = (camera_handle*)pCamHandle;
	pstDepthHandleCb = &pstTofHandle->stDepthHandleCb[0];

	retVal = TOFM_DoDepthCal(pstDepthHandleCb->hTofMod, &stRawData, &stTofDepthData);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("TOFM_DoDepthCal failed, retVal=0x%08x.\n", retVal);
		return -1;
	}

	pstTofDepthDataInfo->timeStamp 			= pstTofRawDataInfo->timeStamp;
	pstTofDepthDataInfo->uiFrameCnt			= pstTofRawDataInfo->uiFrameCnt;
	pstTofDepthDataInfo->frameWidth 		= stTofDepthData.frameWidth;
	pstTofDepthDataInfo->frameHeight 		= stTofDepthData.frameHeight;
	pstTofDepthDataInfo->pDepthData			= stTofDepthData.pDepthData;
	pstTofDepthDataInfo->pfPointData 		= stTofDepthData.pPointData;
	pstTofDepthDataInfo->pPointDataUnfilter = stTofDepthData.pPointDataUnfilter;
	pstTofDepthDataInfo->pu8GrayData		= stTofDepthData.pGrayData;
	pstTofDepthDataInfo->pu16Confidence 	= NULL;
	pstTofDepthDataInfo->pfNoise 			= NULL;
	
	return 0;
}

#ifdef RGBD
static void PrintfLensParameter(TofRgbdLensParameter* pTofLens)
{
	if (NULL == pTofLens)  return;

	const unsigned int nIndex = pTofLens->nIndex;

	if (1 == nIndex)
	{
		TofRgbdLensGeneral* pTmp = &(pTofLens->uParam.general);

		printf("Lens Paramter (general):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   p1 = %f.\n", pTmp->p1);
		printf(">>   p2 = %f.\n", pTmp->p2);
		printf(">>   k3 = %f.\n", pTmp->k3);
	}
	else if (2 == nIndex)
	{
		TofRgbdLensFishEye* pTmp = &(pTofLens->uParam.fishEye);

		printf("Lens Paramter (fishEye):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   k3 = %f.\n", pTmp->k3);
		printf(">>   k4 = %f.\n", pTmp->k4);
	}
	else
	{
		printf("Lens Paramter (index=%u):...............................\n", nIndex);
		printf(">>   unknown, not supported.\n");
	}
}

int TofRgbdSdkInit(HTOFRGBD *ppstRgbdHandle, const char *pcRgbdCalibPath)
{
	TofRgbdParameter struParameters;
	TOFRGBDRET retVal = TOFRGBDRET_FAILED;
	const std::string strModuleName = SELECT_MODULE_NAME;//模组型号
	const TOFRGBD_GUEST_ID guestID = TOFRGBD_GUEST_ID_DEF;//客户识别号

	const unsigned int nTofWidth = RAW_WIDTH;//TOF数据宽
	const unsigned int nTofHeight = ACTIVE_HEIGHT;//TOF数据高
	const unsigned int nRgbWidth = RGB_VPS_OUT_WIDTH;//RGB数据宽
	const unsigned int nRgbHeight = RGB_VPS_OUT_HEIGHT;//RGB数据高

	if (!ppstRgbdHandle || !pcRgbdCalibPath)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}
	
	printf("SDK Version: %s.\n", TOFRGBD_GetSDKVersion());

	CBuf calibData, pointCloudData, grayData, rgbData;
	std::string strCalibFile = pcRgbdCalibPath;
	
	if (!ReadFile(strCalibFile, calibData))
	{	printf("[%s] Read RGBD calib data failed!, path = %s\n", __func__, pcRgbdCalibPath);
		return -1;
	}
	
	TofRgbdHandleParam struInputParam;
	memset(&struInputParam, 0, sizeof(struInputParam));
	strncpy(struInputParam.szModuleName, strModuleName.c_str(), sizeof(struInputParam.szModuleName) - 1);
	struInputParam.guestID = guestID;

	struInputParam.pRgbdCalibData = calibData.GetBuf();
	struInputParam.nRgbdCalibDataLen = 512;		// TODO

	struInputParam.nTofWidth = nTofWidth;
	struInputParam.nTofHeight = nTofHeight;

	struInputParam.nRgbWidth = nRgbWidth;
	struInputParam.nRgbHeight = nRgbHeight;

	HTOFRGBD hTofRgbd = TOFRGBD_CreateHandle(&struInputParam);
	if (NULL == hTofRgbd)
	{
		printf("TOFRGBD_CreateHandle failed.\n");
		return -1;
	}

	//获取并打印RGB内参
	memset(&struParameters, 0, sizeof(struParameters));
	struParameters.type = TOF_RGBD_PARAM_RgbCameraLensParam;
	retVal = TOFRGBD_GetParameters(hTofRgbd, &struParameters);
	if (TOFRGBDRET_SUCCESS == retVal)
	{
		printf(">> RGB LensParam:\n");
		PrintfLensParameter(&struParameters.uParam.lensParam);
	}
	else
	{
		printf("TOFRGBD_GetParameters(TOF_RGBD_PARAM_RgbCameraLensParam) failed, retVal=0x%08x.\n", retVal);
	}

	*ppstRgbdHandle = hTofRgbd;
	return 0;
}


int TofRgbdSdkUnit(HTOFRGBD pstRgbdHandle)
{
#ifdef RGBD
	if (pstRgbdHandle != NULL)
	{
		TOFRGBD_CloseHandle(pstRgbdHandle);
	}

	return 0;
#endif
}


int TofRgbdProcess(void *pCamHandle, TofRgbdInputData* pDataIn, TofRgbdOutputData* pDataOut)
{
#ifdef RGBD
	camera_handle *pstTofHandle;

	if (!pCamHandle || !pDataIn || !pDataOut)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstTofHandle = (camera_handle*)pCamHandle;

	const TOFRGBDRET retVal = TOFRGBD_DoCal(pstTofHandle->apstRgbdHandle[0], pDataIn, pDataOut);
	if (TOFRGBDRET_SUCCESS != retVal)
	{
		printf("TOFRGBD_DoCal failed, retVal=0x%08x.\n", retVal);
		return -1;
	}

	return 0;
#endif
}
#endif
