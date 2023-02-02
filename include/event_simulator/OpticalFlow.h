#pragma once

#include <opencv2/opencv.hpp>

/**
 * @brief automatically selecting of the coarsest scale
 *
 * @param imgwidth Width of the image
 * @param fratio Flow ratio
 * @param patchsize Patch size
 * @return The coarsest scale
 */
int autoFirstScaleSelect(int imgwidth, int fratio, int patchsize);

/**
 * @brief Construct image pyramide used for optical flow calulation
 *        Taken and adjusted from: https://github.com/tikroeger/OF_DIS
 */
void constructImgPyramide(const cv::Mat &img_ao_fmat, cv::Mat *img_ao_fmat_pyr,
                          cv::Mat *img_ao_dx_fmat_pyr,
                          cv::Mat *img_ao_dy_fmat_pyr, const float **img_ao_pyr,
                          const float **img_ao_dx_pyr,
                          const float **img_ao_dy_pyr, const int lv_f,
                          const int rpyrtype, const bool getgrad,
                          const int imgpadding);
