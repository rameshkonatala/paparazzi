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

// To print desired variable values in the command window
#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[ourmainf->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Variables declaration

//Color count variable. Has been used in our_avoider.c
float color_count = 0;
float color_count_right = 0;
float color_count_left = 0;

// Size of rectangle where pixels are counted

int hrec = 50; //height in pixels
int wrec = 200; //width in pixels

// Horizontal Size of the lateral rectangles for comparison

int wrec_sides = 60; //height in pixels

//Define size of the reference grass patch

int hor_sec = 100; //width in pixels
int ver_sec = 100;//height pixels

//Define size of the region where algorithm is run
int w_algo = 300; //width
int h_algo = 100; //height

//Define picture middle coordinates
int hor_mid;   //mid width
int ver_mid;   //mid height

//Histogram Backprojection specifications //??? TRY TO USE Y CHANNEL AS WELL ????
int ubins = 256, vbins = 256;
int histSize[] = {ubins, vbins};
float uranges[] = { 0, 255 };
float vranges[] = { 0, 255 };
int channels[] = {1,2};
const float* ranges[] = { uranges, vranges };

//Extra storage matrices used
Mat element, imgsec, imgsec_right, imgsec_left, algo_img, imgref1;
MatND backproj;MatND hist;

//to store the no. of rows and cols
int rows,cols;

// Use this to print variables and debug
// VERBOSE_PRINT("%d,%d,%d,%d", 0,hor_mid-hor_sec/2, ver_sec, hor_sec);

//Function
int opencv_ourmainf(char *raw_img_data, int width, int height)
{
    // Create a new image, using the original bebop image.
    Mat imgmain(height, width, CV_8UC2, raw_img_data);
    Mat imgref(imgshot.h, imgshot.w, CV_8UC2, (char *) imgshot.buf);

    //No. of rows and cols in the matrices
    rows = imgref.rows;
    cols = imgref.cols;
    //Determine matrix middle indices
    hor_mid = rows/2; //mid width
    ver_mid = cols/2; //mid height

    //Cut out the grass patch from the reference image
    cv::Rect myROI(0,hor_mid-hor_sec/2, ver_sec, hor_sec);
    imgref = imgref(myROI);

    //Select section of the main image to be processed
    cv::Rect algo_box(0,hor_mid-w_algo/2, h_algo, w_algo);
    algo_img = imgmain(algo_box);

    //No. of rows and cols in the image section
    rows = algo_img.rows;
    cols = algo_img.cols;
    //Determine matrix middle indices
    hor_mid = rows/2; //mid width
    ver_mid = cols/2; //mid height


    //Convert color from YUV422 to opencv compatible YUV
  
    cvtColor(algo_img, algo_img, CV_YUV2RGB_Y422);
    cvtColor(algo_img, algo_img, CV_RGB2YUV);
    cvtColor(imgref, imgref, CV_YUV2RGB_Y422);
    cvtColor(imgref, imgref, CV_RGB2YUV);
  

   //Histogram Calculation (Only using U and V channels)

   calcHist( &imgref, 1, channels, Mat(), // do not use mask
                 hist, 2, histSize, ranges,
                 true, // the histogram is uniform
                 false );
   normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

  //Histogram Backprojection

   calcBackProject( &algo_img, 1, channels, hist, backproj, ranges, 10, true );

   //Blurring to remove noise

   element = getStructuringElement( MORPH_ELLIPSE, Size( 5,5 ));
   filter2D(backproj, backproj, -1, element);

   //thresholding (Makes the image binary => Pixel values either 0 or 255)
   threshold(backproj, algo_img, 60, 255, 0);

   //Define the rectangle where pixels are counted

   cv::Rect recta(0,hor_mid-wrec/2, hrec,wrec);
   imgsec = algo_img(recta);

   //Define the rectangles where pixels are counted at right and left sides

   cv::Rect recta_right(0,hor_mid-wrec/2, hrec,wrec_sides);
   cv::Rect recta_left(0,hor_mid+wrec/2-wrec_sides, hrec,wrec_sides);

   imgsec_right = algo_img(recta_right);
   imgsec_left  = algo_img(recta_left);

   //Count no. of white pixels
   color_count = countNonZero(imgsec);
   color_count_right = countNonZero(imgsec_right);
   color_count_left = countNonZero(imgsec_left);

   //White pixel ratio inside the rectangle
   color_count = color_count/(imgsec.rows*imgsec.cols);
   color_count_right = color_count_right/(imgsec_right.rows*imgsec_right.cols);
   color_count_left = color_count_left/(imgsec_left.rows*imgsec_left.cols);

   //Draw the rectangle on the main image
   cv::rectangle(algo_img,recta,Scalar(0,0,0),2);
   cv::rectangle(algo_img,recta_right,Scalar(0,0,0),2);
   cv::rectangle(algo_img,recta_left,Scalar(0,0,0),2);
   //cv::rectangle(imgmain,myROI,Scalar(255,255,255),1);
    
   // grayscale_opencv_to_yuv422(imgmain, raw_img_data, imgmain.cols, imgmain.rows);
    

   //Write the new image over the old image
   rows = imgmain.rows;
   cols = imgmain.cols;
   hor_mid = rows/2; //mid width
   ver_mid = cols/2; //mid height
   int i, j, k=0;
   uchar *p;
   int index_img = 0;
   for (i = 0; i < rows; ++i)
   {
       //only write the pixel values where the algorithm processes the image
       if(i>=hor_mid-w_algo/2  && i<=hor_mid+w_algo/2)
       {
        p = algo_img.ptr<uchar>(k);
        k++;
       }
        for (j = 0; j < cols; j++)
        {
        if(i>=hor_mid-w_algo/2  && i<=hor_mid+w_algo/2 && j>=0 && j<=h_algo)
        {
        	raw_img_data[index_img++] = 127;
        	raw_img_data[index_img++] = p[j];
        }
        else{
        index_img+=2;
        }


        }
      }
      //VERBOSE_PRINT("Done Print \n");

  return 0;
}
