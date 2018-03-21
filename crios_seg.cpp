/*
	 .d8888b.  8888888b.  8888888 .d88888b.   .d8888b.
	d88P  Y88b 888   Y88b   888  d88P   Y88b d88P  Y88b
	888    888 888    888   888  888     888 Y88b.
	888        888   d88P   888  888     888   Y888b.
	888        8888888P     888  888     888      Y88b.
	888    888 888 T88b     888  888     888        888
	Y88b  d88P 888  T88b    888  Y88b. .d88P Y88b  d88P
		Y8888P   888   T88b 8888888  Y88888P     Y8888P
 ============================================================================
	 Name        : crios_seg.cpp
	 Author      : Anderson Ignacio da Silva
	 Version     : 1.0
   Date        : 20/03/2018
	 Copyright   : GPLv3
   libs        : OpenCV 3.1 - Cross compiled for ARM Cortex A9 - Arch ARMv7
	 Description : Image processing program in C++ to segment a grayscale pre-
                 processed image by FPGA. The idea is to read the image that
                 is in the SDRAM wrote by the FPGA and then find the contour
                 of the objects inside. An input image is composed by graysc
                 ale pixels from 0x00 to 0xff and this image must be highlig
                 ht just the ROI color pixels, such as yellow, green, red, b
                 lue that compose the most traffic signs. As the output the
                 program write the regions (countours) in a specific locatio
                 n of the SDRAM.

 ============================================================================
*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>
#include <stdbool.h>

#include "alt_dma.h"
#include "alt_globaltmr.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include "alt_clock_manager.h"
#include "alt_generalpurpose_io.h"
#include "alt_globaltmr.h"
#include "hwlib.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"
#include "crios_seg.h"

using namespace cv;
using namespace std;

Mat image;

vector<vector<Point>> contours;
vector<Vec4i> hierarchy;

Mat filter_red(Mat image_in){
  Mat mask_1, mask_2, imageHSV;
  int th_red = 10;
  cvtColor(image_in, imageHSV, CV_BGR2HSV);
  inRange(imageHSV, Scalar(0, 60, 80), Scalar(th_red, 255, 255), mask_1);
  inRange(imageHSV, Scalar(180-th_red, 150, 150), Scalar(180, 255, 255), mask_2);
  return mask_1|mask_2;
}

unsigned char* getRawImageSDRAM(){
  int fd;
  void *sdram_value;
  if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( (unsigned char*)1 );
  }
  sdram_value = mmap( NULL, SDRAM_SEGMENTED_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_SEGMENTED_BASE );
  // HexDump((unsigned char*)sdram_value, SDRAM_TESTE_SPAN);
  return (unsigned char*)sdram_value;
}

int main(int argc, char** argv) {
#if INPUT_SOURCE == SDRAM
  unsigned char *imgSDRAM;
  int fd;
  Rect bounding_rect;
  void *virtual_base,
       *lw_hps2fpga,
       *lw_fpga2hps;

  if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    cout << "ERROR: could not open \"/dev/mem\"... to map virtual address!\n" << endl;
    return ( 1 );
  }

  virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, (off_t)HW_REGS_BASE );

  if ( virtual_base == MAP_FAILED ){
    cout << "Error: mmap() failed of virtual address...\n" << endl;
    close(fd);
    return ( 1 );
  }

  cout << "Press ESC to exit()..." << endl;

  lw_hps2fpga = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_OUTPUT_2_FPGA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
  lw_fpga2hps = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_INPUT_2_HPS_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

  while (true){
      EN_REQ_WRITE;
      while(WAIT_DONE_WRITE);
      DIS_REQ_WRITE;
      imgSDRAM = getRawImageSDRAM();
      Mat img(480, 640, CV_8UC1, imgSDRAM);
      #ifdef SDRAM_SEG
        findContours( img, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        for( unsigned int i = 0; i < contours.size(); i++ ) {
          // double a = contourArea(contours[i],false);
          bounding_rect = boundingRect(contours[i]);
          rectangle(img, bounding_rect, Scalar(0,255,0),2, 8,0);
          // cout << "\nCountour draw with area = " << a;
        }
      #endif
      namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
      imshow(WINDOW_NAME,img);
      if (waitKey(1) == 27) {
        cout << "ESC key is pressed by user" << endl;
        break;
      }
  }
#else
  #if INPUT_SOURCE == IMG_INPUT
    char *file_input = argv[1];
    image = imread(file_input, CV_LOAD_IMAGE_COLOR);
    Mat filtered = filter_red(image);
    Rect bounding_rect;
    cout << "Press ESC to exit()..." << endl;
    findContours( filtered, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    for( unsigned int i = 0; i < contours.size(); i++ ) {
      double a = contourArea(contours[i],false);
      bounding_rect = boundingRect(contours[i]);
      rectangle(image, bounding_rect, Scalar(0,255,0),2, 8,0);
      cout << "\nCountour draw with area = " << a;
    }
    namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
    imshow(WINDOW_NAME, image);
    while (true) {
      if (waitKey(15) == 27) {
        cout << "ESC key is pressed by user" << endl;
        break;
      }
    }
  #else // #WEBCAM
    VideoCapture *cap = new VideoCapture(0);
    Rect bounding_rect;
    Mat filtered;
    cout << "\nConnect a webcam in the USB..." << endl;
    cout << "\nPress ESC to exit()..." << endl;

    while (true) {
      cap->read(image);
      filtered = filter_red(image);
      findContours( filtered, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
      for( unsigned int i = 0; i < contours.size(); i++ ) {
        double a = contourArea(contours[i],false);
        if (a > 800) {
          bounding_rect = boundingRect(contours[i]);
          rectangle(image, bounding_rect, Scalar(0,255,0),2, 8,0);
        }
        // cout << "\nCountour draw with area = " << a;
      }
      namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
      imshow(WINDOW_NAME, image);
      if (waitKey(15) == 27) {
        cout << "\nESC key is pressed by user" << endl;
        break;
      }
    }
  #endif
#endif
}
