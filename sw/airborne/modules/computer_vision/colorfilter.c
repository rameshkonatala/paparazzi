/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
#include "opencv_image_functions.h"

#include "modules/sliceimage/sliceimage.h"

#include "modules/computer_vision/lib/vision/image.h"

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;
#include "subsystems/abi.h"
// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result
float color_count = 0;
int hor_sec, ver_sec,hor_mid;
int ubins = 256, vbins = 256;
float uranges[] = { 0, 255 };
float vranges[] = { 0, 255 };
int channels[] = {1,2};
const float* ranges[] = { uranges, vranges };
Mat M, element;
MatND backproj;MatND hist;



int rows,cols;
// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
  //Convert image into opencv format
  Mat imgmain(img->h, img->w, CV_8UC2, (char *) img->buf);
  Mat imgref(imgshot->h, imgshot->w, CV_8UC2, (char *) imgshot->buf);

  //Rotate
  rows,cols = imgmain.shape;

  M = getRotationMatrix2D(Point(cols/2,rows/2),270,1);
  imgmain = warpAffine(imgmain,M,(cols,rows));
  imgref = warpAffine(imgref,M,(cols,rows));
  rows,cols = imgmain.shape;

  //rows,cols = imgref.shape;

  // Segment Ref
  hor_sec = int(0.2*cols);
  ver_sec = int(0.15*rows);
  hor_mid = int(cols/2);
  //ver_mid = int(rows/2);
  cv::Rect myROI(hor_mid-hor_sec/2,rows-ver_sec/2, hor_sec,ver_sec/2);
  imgref = imgref(myROI);

  calcHist( &imgref, 1, channels, Mat(), // do not use mask
               hist, 2, histSize, ranges,
               true, // the histogram is uniform
               false );
  normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

  calcBackProject( &imgmain, 1, channels, hist, backproj, ranges, 5, true );
  element = getStructuringElement( MORPH_ELLIPSE, Size( 5,5 ));
  filter2D(backproj, backproj, -1, element);
  threshold(backproj, imgmain, 60, 255, 0);
  cv::Rect recta(hor_mid-hor_sec,frame.rows-ver_sec, 2*hor_sec,ver_sec);
  M = imgmain(recta);
  color_count = countNonZero(M);
  color_count = color_count/(M.rows*M.cols);
  coloryuv_opencv_to_yuv422(imgmain, (char *) img->buf, img->w, img->h);
  
  //Segment the reference image

  	
  /*color_count = image_yuv422_colorfilt(img, img,
                                       color_lum_min, color_lum_max,
                                       color_cb_min, color_cb_max,
                                       color_cr_min, color_cr_max
                                      );*/


  if (COLORFILTER_SEND_OBSTACLE) {
    if (color_count > 20)
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
    }
    else
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
    }
  }

  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}
