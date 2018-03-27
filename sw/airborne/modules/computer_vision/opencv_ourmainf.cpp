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
#include "modules/sliceimage/sliceimage.h"
#include "modules/computer_vision/lib/vision/image.h"

// Variables declaration
float color_count = 0;

// Our comparison rectangle:
int hrec; 
int wrec;

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
int opencv_ourmainf(char *raw_img_data, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat image(height, width, CV_8UC2, raw_img_data);
  Mat imgref(imgshot.h, imgshot.w, CV_8UC2, (char *) imgshot.buf);
  
  //Rotate

  Mat imgmain, rotimgmain, imgref1;
  
  cvtColor(image, imgmain, CV_YUV2RGB_Y422);
  cvtColor(imgmain, imgmain, CV_RGB2YUV);
  cvtColor(imgref, imgref, CV_YUV2RGB_Y422);
  cvtColor(imgref, imgref, CV_RGB2YUV);
  
  // Segment Ref
  rows = imgref.rows;
  cols = imgref.cols;
  
  hor_sec = int(0.3*cols);
  ver_sec = int(0.3*rows);
  hor_mid = int(cols/2);
  ver_mid = int(rows/2);
  
  hrec = 30;
  wrec = 200;
  
  cv::Rect myROI(0,ver_mid-ver_sec/4, hor_sec/2,ver_sec/2);
  
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
    cv::Rect recta(0,ver_mid-wrec/2, hrec,wrec);
    imgsec = imgmain(recta);
    color_count = countNonZero(imgsec);
    color_count = color_count/(imgsec.rows*imgsec.cols);
    cv::rectangle(imgmain,recta,Scalar(0,0,0),1);
    
    grayscale_opencv_to_yuv422(imgmain, raw_img_data, imgmain.cols, imgmain.rows);
    

  return 0;
}
