#include <event_simulator/DISOpticalFlowCalculator.h>
#include <event_simulator/OpticalFlow.h>
#include <oflow.h>

DISOpticalFlowCalculator::DISOpticalFlowCalculator(
    const DISOpticalFlowQuality quality)
    : quality_{quality}, name_{"DISOpticalFlowCalculator"} {}

cv::Mat DISOpticalFlowCalculator::calculateFlow(cv::Mat prev_frame,
                                                cv::Mat frame) {
  cv::Mat img_tmp;
  int rpyrtype, nochannels;

  rpyrtype = CV_32FC1;
  nochannels = 1;

  cv::Mat img_ao_fmat, img_bo_fmat;
  cv::Size sz = prev_frame.size();
  int width_org = sz.width;   // unpadded original image size
  int height_org = sz.height; // unpadded original image size

  // *** Parse rest of parameters, See oflow.h for definitions.
  int lv_f, lv_l, maxiter, miniter, patchsz, patnorm, costfct, tv_innerit,
      tv_solverit, verbosity;
  float mindprate, mindrrate, minimgerr, poverl, tv_alpha, tv_gamma, tv_delta,
      tv_sor;
  bool usefbcon, usetvref;

  mindprate = 0.05;
  mindrrate = 0.95;
  minimgerr = 0.0;
  usefbcon = 0;
  patnorm = 1;
  costfct = 0;
  tv_alpha = 10.0;
  tv_gamma = 10.0;
  tv_delta = 5.0;
  tv_innerit = 1;
  tv_solverit = 3;
  tv_sor = 1.6;
  verbosity = 0; // Default: Plot detailed timings

  int fratio = 5; // For automatic selection of coarsest scale: 1/fratio *
                  // width = maximum expected motion magnitude in image. Set
                  // lower to restrict search space.

  int sel_oppoint = quality_; // Default operating point

  switch (sel_oppoint) {
  case DISOpticalFlowQuality::MEDIUM:
    patchsz = 8;
    poverl = 0.3;
    lv_f = autoFirstScaleSelect(width_org, fratio, patchsz);
    lv_l = std::max(lv_f - 3, 0);
    maxiter = 16;
    miniter = 16;
    usetvref = 1;
    break;
  case DISOpticalFlowQuality::HIGH:
    patchsz = 8;
    poverl = 0.4;
    lv_f = autoFirstScaleSelect(width_org, fratio, patchsz);
    lv_l = std::max(lv_f - 4, 0);
    maxiter = 32;
    miniter = 32;
    usetvref = 1;
    break;
  case DISOpticalFlowQuality::EXTREME:
    patchsz = 8;
    poverl = 0.5;
    lv_f = autoFirstScaleSelect(width_org, fratio, patchsz);
    lv_l = std::max(lv_f - 5, 0);
    maxiter = 64;
    miniter = 64;
    usetvref = 1;
    break;
  case DISOpticalFlowQuality::LOW:
  default:
    patchsz = 4;
    poverl = 0.05;
    lv_f = autoFirstScaleSelect(width_org, fratio, patchsz);
    lv_l = std::max(lv_f - 2, 0);
    maxiter = 8;
    miniter = 8;
    usetvref = 1;
    break;
  }
  // Pad image such that width and height are restless divisible on all
  // scales (except last)
  int padw = 0, padh = 0;
  int scfct = pow(
      2, lv_f); // enforce restless division by this number on coarsest scale
  int div = sz.width % scfct;
  if (div > 0) {
    padw = scfct - div;
  }
  div = sz.height % scfct;
  if (div > 0) {
    padh = scfct - div;
  }
  if (padh > 0 || padw > 0) {
    cv::copyMakeBorder(prev_frame, prev_frame, floor(static_cast<float>(padh) / 2.0f),
                   ceil(static_cast<float>(padh) / 2.0f), floor(static_cast<float>(padw) / 2.0f),
                   ceil(static_cast<float>(padw) / 2.0f), cv::BORDER_REPLICATE);
    cv::copyMakeBorder(frame, frame, floor(static_cast<float>(padh) / 2.0f),
                   ceil(static_cast<float>(padh) / 2.0f), floor(static_cast<float>(padw) / 2.0f),
                   ceil(static_cast<float>(padw) / 2.0f), cv::BORDER_REPLICATE);
  }
  sz = prev_frame.size(); // padded image size, ensures divisibility by 2 on all
                          // scales (except last)

  //  *** Generate scale pyramides
  prev_frame.convertTo(img_ao_fmat, CV_32F); // convert to float
  frame.convertTo(img_bo_fmat, CV_32F);

  const float *img_ao_pyr[lv_f + 1];
  const float *img_bo_pyr[lv_f + 1];
  const float *img_ao_dx_pyr[lv_f + 1];
  const float *img_ao_dy_pyr[lv_f + 1];
  const float *img_bo_dx_pyr[lv_f + 1];
  const float *img_bo_dy_pyr[lv_f + 1];

  cv::Mat img_ao_fmat_pyr[lv_f + 1];
  cv::Mat img_bo_fmat_pyr[lv_f + 1];
  cv::Mat img_ao_dx_fmat_pyr[lv_f + 1];
  cv::Mat img_ao_dy_fmat_pyr[lv_f + 1];
  cv::Mat img_bo_dx_fmat_pyr[lv_f + 1];
  cv::Mat img_bo_dy_fmat_pyr[lv_f + 1];

  constructImgPyramide(img_ao_fmat, img_ao_fmat_pyr, img_ao_dx_fmat_pyr,
                       img_ao_dy_fmat_pyr, img_ao_pyr, img_ao_dx_pyr,
                       img_ao_dy_pyr, lv_f, rpyrtype, 1, patchsz);
  constructImgPyramide(img_bo_fmat, img_bo_fmat_pyr, img_bo_dx_fmat_pyr,
                       img_bo_dy_fmat_pyr, img_bo_pyr, img_bo_dx_pyr,
                       img_bo_dy_pyr, lv_f, rpyrtype, 1, patchsz);

  //  *** Run main optical flow / depth algorithm
  float sc_fct = pow(2, lv_l);

  cv::Mat flowout(sz.height / sc_fct, sz.width / sc_fct,
                  CV_32FC2); // Optical Flow

  OFC::OFClass ofc(
      img_ao_pyr, img_ao_dx_pyr, img_ao_dy_pyr, img_bo_pyr, img_bo_dx_pyr,
      img_bo_dy_pyr,
      patchsz, // extra image padding to avoid border violation check
      (float *)flowout.data, // pointer to n-band output float array
      nullptr, // pointer to n-band input float array of size of first
               // (coarsest) scale, pass as nullptr to disable
      sz.width, sz.height, lv_f, lv_l, maxiter, miniter, mindprate, mindrrate,
      minimgerr, patchsz, poverl, usefbcon, costfct, nochannels, patnorm,
      usetvref, tv_alpha, tv_gamma, tv_delta, tv_innerit, tv_solverit, tv_sor,
      verbosity);

  // *** Resize to original scale, if not run to finest level
  if (lv_l != 0) {
    flowout *= sc_fct;
    cv::resize(flowout, flowout, cv::Size(), sc_fct, sc_fct, cv::INTER_LINEAR);
  }

  // If image was padded, remove padding before saving to file
  flowout =
      flowout(cv::Rect((int)floor((float)padw / 2.0f),
                       (int)floor((float)padh / 2.0f), width_org, height_org));

  return flowout;
}
