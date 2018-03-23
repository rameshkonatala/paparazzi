/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_ourmainf.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

// Additional modules
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "modules/sliceimage/sliceimage.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "subsystems/abi.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[ourmainf->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Variables declaration
float color_count = 0;
int hor_sec, ver_sec,hor_mid, ver_mid;
int ubins = 256, vbins = 256;
int histSize[] = {ubins, vbins};
float uranges[] = { 0, 255 };
float vranges[] = { 0, 255 };
int channels[] = {1,2};
const float* ranges[] = { uranges, vranges };
Mat element, imgsec, imgmain1, imgref1;
MatND backproj;MatND hist;
int rows,cols;


//Function

int opencv_ourmainf(char *img, int width, int height)
{
  // Filter
  //Convert image into opencv format
  Mat image(height, width, CV_8UC2, img);
  Mat imgref(imgshot.h, imgshot.w, CV_8UC2, (char *) imgshot.buf);

  //Rotate

  Mat imgmain, rotimgmain, imgref1;
  //cvtColor(image, imgmain, CV_YUV2GRAY_Y422);
  //cvtColor(imgref, imgref, CV_YUV2GRAY_Y422);
  //int edgeThresh = 35;
  //Canny(imgmain, imgmain, edgeThresh, edgeThresh * 3);
  //
  //cvtColor(image, imgmain, CV_YUV2GRAY_Y422);
  cvtColor(image, imgmain, CV_YUV2RGB_Y422);
  cvtColor(imgmain, imgmain, CV_RGB2YUV);
  cvtColor(imgref, imgref, CV_YUV2RGB_Y422);
  cvtColor(imgref, imgref, CV_RGB2YUV);
  //rows=imgmain.rows;
  //cols=imgmain.cols;

 // cvtColor(image, imgmain, CV_YUV2RGB_Y422);
  //cvtColor(imgmain, imgmain, CV_RGB2YUV);
 // VERBOSE_PRINT("imgmain rows : %d \n imgmain cols : %d \n width : %d \n height : %d", rows, cols, width, height);
  //Point2f imgmain_center(imgmain.cols/2.0F, imgmain.rows/2.0F);
  //Mat M = getRotationMatrix2D(imgmain_center,90,1);
  //warpAffine(imgmain,rotimgmain,M,Size(imgmain.rows, imgmain.cols));
  //rows=rotimgmain.rows;
  //cols=rotimgmain.cols;
  //VERBOSE_PRINT("imgmain rows : %d \n imgmain cols : %d \n width : %d \n height : %d", rows, cols, width, height);
  /*warpAffine(imgref,imgref1,M,Size(cols,rows));
  rows=imgmain1.rows;
  cols=imgmain1.cols;*/

  // Segment Ref
  rows = imgref.rows;
  cols = imgref.cols;


  hor_sec = int(0.3*cols);
  ver_sec = int(0.3*rows);
  hor_mid = int(cols/2);
  ver_mid = int(rows/2);
  cv::Rect myROI(0,ver_mid-ver_sec/4, hor_sec/2,ver_sec/2);
  //cv::Rect myROI(0,0,100,100);
  //VERBOSE_PRINT("hor_sec = %d, ver_mid-ver_sec/2 = %d , ver_mid+ver_sec/2 = %d", hor_sec, ver_mid-ver_sec/2, ver_mid+ver_sec/2);
  imgref = imgref(myROI);

  calcHist( &imgref, 1, channels, Mat(), // do not use mask
               hist, 2, histSize, ranges,
               true, // the histogram is uniform
               false );
  normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

  calcBackProject( &imgmain, 1, channels, hist, backproj, ranges, 10, true );
  element = getStructuringElement( MORPH_ELLIPSE, Size( 5,5 ));
  filter2D(backproj, backproj, -1, element);
  threshold(backproj, imgmain, 60, 255, 0);
  cv::Rect recta(0,ver_mid-ver_sec/2, hor_sec,ver_sec);
  imgsec = imgmain(recta);
  color_count = countNonZero(imgsec);
  color_count = color_count/(imgsec.rows*imgsec.cols);
  cv::rectangle(imgmain,recta,Scalar(0,0,0),5);
  //VERBOSE_PRINT("no. of pixels = %d , color count = %d \n",imgsec.rows*imgsec.cols, color_count);
  //cv::resize(imgmain, imgmain, cv::Size(), 0.6, 0.6);
  //coloryuv_opencv_to_yuv422(imgmain, img, rows, cols);
  //colorrgb_opencv_to_yuv422(image, img, width, height);
  grayscale_opencv_to_yuv422(imgmain, img, imgmain.cols, imgmain.rows);
  //imshow("Final", imgmain);




//  if (COLORFILTER_SEND_OBSTACLE) {
//    if (color_count > 20)
//    {
//      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
//    }
//    else
//    {
//      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
//    }
//  }

  return 0;
}
