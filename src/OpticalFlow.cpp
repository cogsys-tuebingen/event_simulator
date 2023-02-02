#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

int autoFirstScaleSelect(int imgwidth, int fratio, int patchsize) {
  return std::max(
      0, static_cast<int>(std::floor(log2(
             (2.0f * static_cast<float>(imgwidth)) /
             (static_cast<float>(fratio) * static_cast<float>(patchsize))))));
}

void constructImgPyramide(const cv::Mat &img_ao_fmat, cv::Mat *img_ao_fmat_pyr,
                          cv::Mat *img_ao_dx_fmat_pyr,
                          cv::Mat *img_ao_dy_fmat_pyr, const float **img_ao_pyr,
                          const float **img_ao_dx_pyr,
                          const float **img_ao_dy_pyr, const int lv_f, const int rpyrtype,
                          const bool getgrad, const int imgpadding) {

  // Construct image and gradient pyramides
  for (std::size_t i = 0; i <= static_cast<std::size_t>(lv_f); ++i) {
    if (i == 0) // At finest scale: copy directly, for all other: downscale
                // previous scale by .5
    {
      img_ao_fmat_pyr[i] = img_ao_fmat.clone();
    } else {
      resize(img_ao_fmat_pyr[i - 1], img_ao_fmat_pyr[i], cv::Size(), .5, .5,
             cv::INTER_LINEAR);
    }

    img_ao_fmat_pyr[i].convertTo(img_ao_fmat_pyr[i], rpyrtype);

    if (getgrad) {
      cv::Sobel(img_ao_fmat_pyr[i], img_ao_dx_fmat_pyr[i], CV_32F, 1, 0, 3,
                1 / 8.0, 0, cv::BORDER_DEFAULT);
      cv::Sobel(img_ao_fmat_pyr[i], img_ao_dy_fmat_pyr[i], CV_32F, 0, 1, 3,
                1 / 8.0, 0, cv::BORDER_DEFAULT);
      img_ao_dx_fmat_pyr[i].convertTo(img_ao_dx_fmat_pyr[i], CV_32F);
      img_ao_dy_fmat_pyr[i].convertTo(img_ao_dy_fmat_pyr[i], CV_32F);
    }
  }

  // pad images
  for (std::size_t i = 0; i <= static_cast<std::size_t>(lv_f); ++i) {
    cv::copyMakeBorder(
        img_ao_fmat_pyr[i], img_ao_fmat_pyr[i], imgpadding, imgpadding,
        imgpadding, imgpadding,
        cv::BORDER_REPLICATE); // Replicate border for image padding
    img_ao_pyr[i] = (float *)img_ao_fmat_pyr[i].data;

    if (getgrad) {
      cv::copyMakeBorder(img_ao_dx_fmat_pyr[i], img_ao_dx_fmat_pyr[i],
                         imgpadding, imgpadding, imgpadding, imgpadding,
                         cv::BORDER_CONSTANT,
                         0); // Zero padding for gradients
      cv::copyMakeBorder(img_ao_dy_fmat_pyr[i], img_ao_dy_fmat_pyr[i],
                         imgpadding, imgpadding, imgpadding, imgpadding,
                         cv::BORDER_CONSTANT, 0);

      img_ao_dx_pyr[i] = (float *)img_ao_dx_fmat_pyr[i].data;
      img_ao_dy_pyr[i] = (float *)img_ao_dy_fmat_pyr[i].data;
    }
  }
}
