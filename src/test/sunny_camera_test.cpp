// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera_cfg.h"
#include "camera_control.h"
#include "tof_depth_process.h"

static int giExit = 0;
bool enable_frame_drop___ = false;

unsigned long long GetTimestampDiff(unsigned long long u64TofTs,
                                    unsigned long long u64RgbTs) {
  if (u64TofTs >= u64RgbTs) {
    return (u64TofTs - u64RgbTs);
  } else {
    return (u64RgbTs - u64TofTs);
  }
}

void ST_HandleSig(int signo) {
  if (signo == SIGINT) {
    printf("catch Ctrl + C, exit normally\n");

    giExit = 1;
  }
}

#define T_RGB (100 * 1000)

int main(int argc, char *argv[]) {
  std::signal(SIGINT, ST_HandleSig);

  CAM_PARAM_S stTofCamParam;
  CAM_PARAM_S stRgbCamParam;

  void *pTofHandle = NULL;
  void *pRgbHandle = NULL;

  memset(&stTofCamParam, 0, sizeof(stTofCamParam));
  stTofCamParam.eCamType = CAM_TYPE_TOF_RGBD;
  pTofHandle = OpenCamera(&stTofCamParam);
  if (!pTofHandle) {
    printf("open tof camera failed\n");
    return -1;
  }
  printf("open tof camera succeeded\n");

  memset(&stRgbCamParam, 0, sizeof(stRgbCamParam));
  stRgbCamParam.eCamType = CAM_TYPE_RGB;
  pRgbHandle = OpenCamera(&stRgbCamParam);
  if (!pRgbHandle) {
    printf("open rgb camera failed\n");
    return -1;
  }
  printf("open rgb camera succeeded\n");

  CameraStreamON(pTofHandle);
  CameraStreamON(pRgbHandle);

  int iRet = 0;

  IMAGE_DATA_INFO_S stTofRawData;
  IMAGE_DATA_INFO_S stRgbYuvData;
  memset(&stTofRawData, 0, sizeof(stTofRawData));
  memset(&stRgbYuvData, 0, sizeof(stRgbYuvData));
  unsigned short *pu16TofRawData;

  TOF_DEPTH_DATA_INFO_S stTmpDepthInfo = {0};
  TofRgbdInputData stTofRgbdInput;
  TofRgbdOutputData stTofRgbdOut;

  while (!giExit) {
    unsigned long long u64TsDiff;

  GET_TOF:
    stTofRawData.ePixelFormat = PIXEL_FORMAT_RAW;
    iRet = GetImageData(pTofHandle, &stTofRawData);
    if (iRet) {
      usleep(10 * 1000);
      continue;
    }
    pu16TofRawData = (unsigned short *)stTofRawData.pucImageData;
    stTofRawData.uiFrameCnt = pu16TofRawData[3] & 0xFFF;

    u64TsDiff =
        GetTimestampDiff(stTofRawData.timeStamp, stRgbYuvData.timeStamp);
    if (u64TsDiff < T_RGB / 2) {
      goto SYNC_OUT;
    } else {
      if (stTofRawData.timeStamp < stRgbYuvData.timeStamp) {
        goto GET_TOF;
      } else {
        goto GET_RGB;
      }
    }

  GET_RGB:
    stRgbYuvData.ePixelFormat = PIXEL_FORMAT_YUV;
    iRet = GetImageData(pRgbHandle, &stRgbYuvData);
    if (iRet) {
      usleep(10 * 1000);
      continue;
    }

    u64TsDiff =
        GetTimestampDiff(stTofRawData.timeStamp, stRgbYuvData.timeStamp);
    if (u64TsDiff < T_RGB / 2) {
      goto SYNC_OUT;
    } else {
      if (stTofRawData.timeStamp < stRgbYuvData.timeStamp) {
        goto GET_TOF;
      } else {
        goto GET_RGB;
      }
    }

  SYNC_OUT:
    iRet = TofDepthProcess(pTofHandle, &stTofRawData, &stTmpDepthInfo);
    if (iRet) {
      printf("[%s] TofDepthProcess failed\n", __func__);
      continue;
    }

    cv::Mat nv12(RGB_VPS_OUT_HEIGHT * 3 / 2, RGB_VPS_OUT_WIDTH, CV_8UC1,
                 const_cast<char *>(reinterpret_cast<const char *>(
                     stRgbYuvData.pucImageData)));
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);

    stTofRgbdInput.pPointCloud =
        (TofRgbdPointCloud *)stTmpDepthInfo.pfPointData;
    stTofRgbdInput.pGray = stTmpDepthInfo.pu8GrayData;
    stTofRgbdInput.pRgb = bgr.data;
    stTofRgbdInput.nRgbLen = RGB_SIZE;

    iRet = TofRgbdProcess(pTofHandle, &stTofRgbdInput, &stTofRgbdOut);
    if (iRet) {
      printf("[%s] TofRgbdProcess failed\n", __func__);
      continue;
    }

    // if (stTofRawData.uiFrameCnt == 50) {
    //   std::string strSaveDir = "./";
    //   std::string save_path = "./";
    //   HandleDepthData(0, stTofRawData.uiFrameCnt, strSaveDir, "t00p11a",
    //                   &stTmpDepthInfo);
    //   HandleTofRgbdOutputData(stTofRawData.uiFrameCnt, save_path,
    //                           &stTofRgbdInput, &stTofRgbdOut);
    // }
  }

  CloseCamera(pTofHandle);
  CloseCamera(pRgbHandle);
  printf("[%s] Sunny Camera End\n", __func__);
  return 0;
}