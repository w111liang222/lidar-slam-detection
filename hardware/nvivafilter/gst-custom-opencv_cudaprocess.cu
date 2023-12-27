/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <sys/syscall.h>
#include <cuda.h>
#include "cudaEGL.h"

#define gettid() ((pid_t)syscall(SYS_gettid))

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/cudawarping.hpp"

#if defined(__cplusplus)
extern "C" void Handle_EGLImage (EGLImageKHR image);
extern "C" {
#endif

typedef enum {
  COLOR_FORMAT_Y8 = 0,
  COLOR_FORMAT_U8_V8,
  COLOR_FORMAT_RGBA,
  COLOR_FORMAT_NONE
} ColorFormat;

typedef struct {
  /**
  * cuda-process API
  *
  * @param image   : EGL Image to process
  * @param userPtr : point to user alloc data, should be free by user
  */
  void (*fGPUProcess) (EGLImageKHR image, void ** userPtr);

  /**
  * pre-process API
  *
  * @param sBaseAddr  : Mapped Surfaces(YUV) pointers
  * @param smemsize   : surfaces size array
  * @param swidth     : surfaces width array
  * @param sheight    : surfaces height array
  * @param spitch     : surfaces pitch array
  * @param sformat    : surfaces format array
  * @param nsurfcount : surfaces count
  * @param userPtr    : point to user alloc data, should be free by user
  */
  void (*fPreProcess)(void **sBaseAddr,
                      unsigned int *smemsize,
                      unsigned int *swidth,
                      unsigned int *sheight,
                      unsigned int *spitch,
                      ColorFormat *sformat,
                      unsigned int nsurfcount,
                      void ** userPtr);

  /**
  * post-process API
  *
  * @param sBaseAddr  : Mapped Surfaces(YUV) pointers
  * @param smemsize   : surfaces size array
  * @param swidth     : surfaces width array
  * @param sheight    : surfaces height array
  * @param spitch     : surfaces pitch array
  * @param sformat    : surfaces format array
  * @param nsurfcount : surfaces count
  * @param userPtr    : point to user alloc data, should be free by user
  */
  void (*fPostProcess)(void **sBaseAddr,
                      unsigned int *smemsize,
                      unsigned int *swidth,
                      unsigned int *sheight,
                      unsigned int *spitch,
                      ColorFormat *sformat,
                      unsigned int nsurfcount,
                      void ** userPtr);
} CustomerFunction;

void init (CustomerFunction * pFuncs);

#if defined(__cplusplus)
}
#endif


/**
  * Dummy custom pre-process API implematation.
  * It just access mapped surface userspace pointer &
  * memset with specific pattern modifying pixel-data in-place.
  *
  * @param sBaseAddr  : Mapped Surfaces pointers
  * @param smemsize   : surfaces size array
  * @param swidth     : surfaces width array
  * @param sheight    : surfaces height array
  * @param spitch     : surfaces pitch array
  * @param nsurfcount : surfaces count
  */
static void
pre_process (void **sBaseAddr,
                unsigned int *smemsize,
                unsigned int *swidth,
                unsigned int *sheight,
                unsigned int *spitch,
                ColorFormat  *sformat,
                unsigned int nsurfcount,
                void ** usrptr)
{
   printf ("pre-process %dx%d size %d\n", *swidth, *sheight, *smemsize);
}

/**
  * Dummy custom post-process API implematation.
  * It just access mapped surface userspace pointer &
  * memset with specific pattern modifying pixel-data in-place.
  *
  * @param sBaseAddr  : Mapped Surfaces pointers
  * @param smemsize   : surfaces size array
  * @param swidth     : surfaces width array
  * @param sheight    : surfaces height array
  * @param spitch     : surfaces pitch array
  * @param nsurfcount : surfaces count
  */
static void
post_process (void **sBaseAddr,
                unsigned int *smemsize,
                unsigned int *swidth,
                unsigned int *sheight,
                unsigned int *spitch,
                ColorFormat  *sformat,
                unsigned int nsurfcount,
                void ** usrptr)
{
   printf ("post-process %dx%d size %d\n", *swidth, *sheight, *smemsize);
}


#define CONFIG_FILE_PATH "/tmp/camera_config"

typedef struct {
  cv::cuda::Stream stream;
  cv::cuda::GpuMat xmap;
  cv::cuda::GpuMat ymap;

  bool is_wrap;
  cv::Size out_size;
  cv::Size perspective_size;
  cv::Mat homography;
  cv::Mat affine;
} Config_t;

static std::map<pid_t, std::string> name_map;
static std::map<std::string, Config_t> config_map;

void clear() {
  name_map.clear();
  config_map.clear();
}

std::vector<std::string> get_config() {
  std::ifstream f(CONFIG_FILE_PATH);
  std::string s;
  std::vector<std::string> result;
  while (getline(f, s, ' ')) {
      result.push_back(s);
  }
  std::remove(CONFIG_FILE_PATH);
  return result;
}

void get_cv_remap(std::vector<std::string> config, cv::cuda::GpuMat &gpu_xmap, cv::cuda::GpuMat &gpu_ymap, cv::Size &output_size) {
  int   w  = std::stoi(config[1]);
  int   h  = std::stoi(config[2]);
  float fx = std::stof(config[3]);
  float fy = std::stof(config[4]);
  float cx = std::stof(config[5]);
  float cy = std::stof(config[6]);
  float k1 = std::stof(config[7]);
  float k2 = std::stof(config[8]);
  float p1 = std::stof(config[9]);
  float p2 = std::stof(config[10]);
  int fisheye = std::stoi(config[11]);

  output_size = cv::Size(w, h);

  /* Initialize maps from CPU */
  cv::Mat xmap(h, w, CV_32FC1);
  cv::Mat ymap(h, w, CV_32FC1);

   //fill matrices
  cv::Mat cam(3, 3, cv::DataType<float>::type);
  cam.at<float>(0, 0) = fx;
  cam.at<float>(0, 1) = 0.0f;
  cam.at<float>(0, 2) = cx;

  cam.at<float>(1, 0) = 0.0f;
  cam.at<float>(1, 1) = fy;
  cam.at<float>(1, 2) = cy;

  cam.at<float>(2, 0) = 0.0f;
  cam.at<float>(2, 1) = 0.0f;
  cam.at<float>(2, 2) = 1.0f;

  cv::Mat dist(4, 1, cv::DataType<float>::type);
  dist.at<float>(0, 0) = k1;
  dist.at<float>(1, 0) = k2;
  dist.at<float>(2, 0) = p1;
  dist.at<float>(3, 0) = p2;

  if (fisheye == 1) {
    cv::fisheye::initUndistortRectifyMap(cam, dist, cv::Mat(), cam, cv::Size(w, h), CV_32FC1, xmap, ymap);
  }
  else {
    cv::initUndistortRectifyMap(cam, dist, cv::Mat(), cam, cv::Size(w, h), CV_32FC1, xmap, ymap);
  }

  /* upload to GpuMats */
  gpu_xmap.upload(xmap);
  gpu_ymap.upload(ymap);

  printf("init camera distortion: %s, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f\n",
                                  config[0].c_str(), w, h, fx, fy, cx, cy, k1, k2, p1, p2);
}

int get_wrap_config(std::vector<std::string> config, cv::Mat &perspective, cv::Mat &affine, cv::Size &perspect_size) {
  perspective = cv::Mat(3, 3, cv::DataType<float>::type);
  affine = cv::Mat(2, 3, cv::DataType<float>::type);

  int do_wrap = std::stoi(config[12]);
  if (do_wrap != 0) {
    int perspective_width = std::stoi(config[13]);
    int perspective_height = std::stoi(config[14]);
    int affine_width = std::stoi(config[15]);
    int affine_height = std::stoi(config[16]);

    perspect_size = cv::Size(perspective_width, perspective_height);

    perspective.at<float>(0, 0) = std::stof(config[17]);
    perspective.at<float>(0, 1) = std::stof(config[18]);
    perspective.at<float>(0, 2) = std::stof(config[19]);

    perspective.at<float>(1, 0) = std::stof(config[20]);
    perspective.at<float>(1, 1) = std::stof(config[21]);
    perspective.at<float>(1, 2) = std::stof(config[22]);

    perspective.at<float>(2, 0) = std::stof(config[23]);
    perspective.at<float>(2, 1) = std::stof(config[24]);
    perspective.at<float>(2, 2) = std::stof(config[25]);

    affine.at<float>(0, 0) = std::stof(config[26]);
    affine.at<float>(0, 1) = std::stof(config[27]);
    affine.at<float>(0, 2) = std::stof(config[28]);

    affine.at<float>(1, 0) = std::stof(config[29]);
    affine.at<float>(1, 1) = std::stof(config[30]);
    affine.at<float>(1, 2) = std::stof(config[31]);

    printf("init camera wrap: %s, %d, %d, %d, %d, %d\n",
                              config[0].c_str(), do_wrap, perspective_width, perspective_height, affine_width, affine_height);
    printf("perspective: %s, [%f, %f, %f], [%f, %f, %f], [%f, %f, %f]\n",
                        config[0].c_str(), perspective.at<float>(0, 0), perspective.at<float>(0, 1), perspective.at<float>(0, 2),
                                            perspective.at<float>(1, 0), perspective.at<float>(1, 1), perspective.at<float>(1, 2),
                                            perspective.at<float>(2, 0), perspective.at<float>(2, 1), perspective.at<float>(2, 2));
    printf("affine: %s, [%f, %f, %f], [%f, %f, %f]\n",
                        config[0].c_str(), affine.at<float>(0, 0), affine.at<float>(0, 1), affine.at<float>(0, 2),
                                            affine.at<float>(1, 0), affine.at<float>(1, 1), affine.at<float>(1, 2));
  } else {
    printf("no panorama camera config\n");
  }
  return do_wrap;
}
void cv_process_RGBA(void *pdata, int32_t width, int32_t height)
{
    pid_t token = gettid();
    Config_t cfg;
    std::string name = "";
    if (name_map.find(token) == name_map.end()) {
      std::vector<std::string> config = get_config();
      if (config.size() == 0) {
        std::cerr << "Error, No camera config found" << std::endl;
        return;
      }
      name = config[0];
      get_cv_remap(config, cfg.xmap, cfg.ymap, cfg.out_size);
      cfg.is_wrap = get_wrap_config(config, cfg.homography, cfg.affine, cfg.perspective_size);

      name_map[token] = name;
      config_map[name] = cfg;
    } else {
      name = name_map[token];
      cfg = config_map[name];
    }

    cv::cuda::GpuMat dst(height, width, CV_8UC4, pdata);
    cv::cuda::GpuMat src;
    dst.copyTo(src, cfg.stream); // copy to src mat
    if (cfg.is_wrap == 0) {
      cv::cuda::remap(src, dst, cfg.xmap, cfg.ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0.f, 0.f, 0.f, 0.f), cfg.stream);
    } else {
      cv::cuda::GpuMat src_map;
      cv::cuda::remap(src, src_map, cfg.xmap, cfg.ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0.f, 0.f, 0.f, 0.f), cfg.stream);
      cv::cuda::GpuMat perspect;
      cv::cuda::warpPerspective(src_map, perspect, cfg.homography, cfg.perspective_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0.f, 0.f, 0.f, 0.f), cfg.stream);
      cv::cuda::warpAffine(perspect, dst, cfg.affine, cfg.out_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0.f, 0.f, 0.f, 0.f), cfg.stream);
    }

    cfg.stream.waitForCompletion();
    // Check
    if(dst.data != pdata)
	    std::cerr << "Error, reallocated buffer for pdata" << std::endl;
}



/**
  * Performs CUDA Operations on egl image.
  *
  * @param image : EGL image
  */
void
gpu_process (EGLImageKHR image, void ** usrptr)
{
  CUresult status;
  CUeglFrame eglFrame;
  CUgraphicsResource pResource = NULL;

  cudaFree(0);
  status = cuGraphicsEGLRegisterImage(&pResource, image, CU_GRAPHICS_MAP_RESOURCE_FLAGS_NONE);

  if (status != CUDA_SUCCESS) {
    printf("cuGraphicsEGLRegisterImage failed : %d \n", status);
    return;
  }

  status = cuGraphicsResourceGetMappedEglFrame( &eglFrame, pResource, 0, 0);
  if (status != CUDA_SUCCESS) {
    printf ("cuGraphicsSubResourceGetMappedArray failed\n");
  }

  status = cuCtxSynchronize();
  if (status != CUDA_SUCCESS) {
    printf ("cuCtxSynchronize failed \n");
  }

  if (eglFrame.frameType == CU_EGL_FRAME_TYPE_PITCH) {
    if (eglFrame.eglColorFormat == CU_EGL_COLOR_FORMAT_ABGR || eglFrame.eglColorFormat == CU_EGL_COLOR_FORMAT_RGBA) {
 	    cv_process_RGBA(eglFrame.frame.pPitch[0], eglFrame.width, eglFrame.height);
    } else if (eglFrame.eglColorFormat == CU_EGL_COLOR_FORMAT_YUV420_SEMIPLANAR) {
      printf ("Invalid eglcolorformat NV12\n");
    } else
      printf ("Invalid eglcolorformat %d\n", eglFrame.eglColorFormat);
  }

  status = cuCtxSynchronize();
  if (status != CUDA_SUCCESS) {
    printf ("cuCtxSynchronize failed after memcpy \n");
  }

  status = cuGraphicsUnregisterResource(pResource);
  if (status != CUDA_SUCCESS) {
    printf("cuGraphicsEGLUnRegisterResource failed: %d \n", status);
  }
}

extern "C" void
init (CustomerFunction * pFuncs)
{
  pFuncs->fPreProcess = pre_process;
  pFuncs->fGPUProcess = gpu_process;
  pFuncs->fPostProcess = post_process;
}

extern "C" void
deinit (void)
{

}