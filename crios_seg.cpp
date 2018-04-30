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

//#include "alt_dma.h"
//#include "alt_globaltmr.h"
//#include "socal/hps.h"
//#include "socal/socal.h"
//#include "alt_clock_manager.h"
//#include "alt_generalpurpose_io.h"
//#include "alt_globaltmr.h"
//#include "hwlib.h"
//#include "socal/alt_gpio.h"
//#include "hps_0.h"
#include "crios_seg.h"

using namespace cv;
using namespace std;

Mat image;

Mat filter_yellow(Mat image_in){
  Mat mask_1, mask_2, imageHSV;
  Mat yellow_range;
  cvtColor(image_in, imageHSV, CV_BGR2HSV);
  inRange(imageHSV, Scalar(23,0,0), Scalar(40,255,255), yellow_range);
  return yellow_range;
}

Mat filter_red(Mat image_in){
  Mat mask_1, mask_2, imageHSV;
  Mat lower_red_hue_range;
 	Mat upper_red_hue_range;
  cvtColor(image_in, imageHSV, CV_BGR2HSV);
  inRange(imageHSV, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range);
 	inRange(imageHSV, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_red_hue_range);
  return lower_red_hue_range|upper_red_hue_range;
}

Mat filter_green(Mat image_in){
  Mat mask_1, mask_2, imageHSV;
  Mat lower_red_hue_range;
 	Mat upper_red_hue_range;
  int sensitivity = 15;
  cvtColor(image_in, imageHSV, CV_BGR2HSV);
  inRange(imageHSV, Scalar(44,54,63), Scalar(71,255,255), lower_red_hue_range);
  return lower_red_hue_range|upper_red_hue_range;
}

Mat detectRED(Mat image_in){
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  Rect bounding_rect[10000];
  Mat filtered_red, image_out;

  image_out = image_in;
  filtered_red = filter_red(image_out);
  findContours( filtered_red, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  if (contours.size() > 0){
    for( unsigned int i = 0; i < contours.size(); i++ ) {
      double a = contourArea(contours[i],false);
      if (a > 18000 && a < 32000) {
        bounding_rect[i] = boundingRect(contours[i]);
        rectangle(image_out, bounding_rect[i], Scalar(0,0,255),2, 8,0);
        putText(image_out,
                "PARE Vermelha",
                Point(bounding_rect[i].x,bounding_rect[i].y+20+bounding_rect[i].height), // Coordinates
                FONT_HERSHEY_COMPLEX_SMALL, // Font
                1.0, // Scale. 2.0 = 2x bigger
                Scalar(0,0,255), // Color
                1, // Thickness
                CV_AA); // Anti-alias
      }
    }
  }
  return image_out;
}

Mat detectYELLOW(Mat image_in){
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  Rect bounding_rect[10000];
  Mat filtered_yellow, image_out;

  image_out = image_in;
  filtered_yellow = filter_yellow(image_out);
  findContours( filtered_yellow, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  if (contours.size() > 0){
    for( unsigned int i = 0; i < contours.size(); i++ ) {
      double a = contourArea(contours[i],false);

      int h=bounding_rect[i].height;
      if (a > 18000 && a < 23000) {
        // +to_string(a)
        bounding_rect[i] = boundingRect(contours[i]);
        rectangle(image_out, bounding_rect[i], Scalar(0,255,255),2, 8,0);
        putText(image_out,
                "PARE Amarela",
                Point(bounding_rect[i].x,bounding_rect[i].y+20+bounding_rect[i].height), // Coordinates
                FONT_HERSHEY_COMPLEX_SMALL, // Font
                1.0, // Scale. 2.0 = 2x bigger
                Scalar(0,255,255), // Color
                1, // Thickness
                CV_AA); // Anti-alias
      }
    }
  }
  return image_out;
}

Mat detectGREEN(Mat image_in){
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  Rect bounding_rect[10000];
  Mat filtered_green, image_out;

  image_out = image_in;
  filtered_green = filter_green(image_out);
  findContours( filtered_green, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
  if (contours.size() > 0){
    for( unsigned int i = 0; i < contours.size(); i++ ) {
      double a = contourArea(contours[i],false);

      int h=bounding_rect[i].height;
      if (a > 5000) {
        bounding_rect[i] = boundingRect(contours[i]);
        rectangle(image_out, bounding_rect[i], Scalar(0,255,0),2, 8,0);
        putText(image_out,
                "PLACA Informacoes",
                Point(bounding_rect[i].x,bounding_rect[i].y+20+bounding_rect[i].height), // Coordinates
                FONT_HERSHEY_COMPLEX_SMALL, // Font
                1.0, // Scale. 2.0 = 2x bigger
                Scalar(0,255,0), // Color
                1, // Thickness
                CV_AA); // Anti-alias
      }
    }
  }
  return image_out;
}

unsigned char* getRawImageSDRAM(int fd){
  // int fd;
  void *sdram_value;
  // if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
  //   printf( "ERROR: could not open \"/dev/mem\"...\n" );
  //   return( (unsigned char*)1 );
  // }
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

  Mat src, erosion_dst, dilation_dst;

  while (true){
    // printf("\nPedindo requisicao input_2_hps...");
    // EN_REQ_WRITE;
    // while(1) {
    //   printf("\nAguardando write done [%x]...",INPUT_FPGA_BIT0);
    //   if (INPUT_FPGA_BIT0 == 1)
    //     break;
    //   sleep(1);
    // }
    // printf("OK..pedindo outra imagem");
    // sleep(1);

    //printf("\nRequisitando imagem...");
    EN_REQ_WRITE;
    while(WAIT_DONE_WRITE);
    DIS_REQ_WRITE;
    imgSDRAM = getRawImageSDRAM(fd);
    Mat img(480, 640, CV_8UC1, imgSDRAM);
    #ifdef SDRAM_SEG
      findContours( img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
      // for( unsigned int i = 0; i < contours.size(); i++ ) {
      //   // double a = contourArea(contours[i],false);
      //   bounding_rect = boundingRect(contours[i]);
      //   rectangle(img, bounding_rect, Scalar(0,255,0),2, 8,0);
      //   // cout << "\nCountour draw with area = " << a;
      // }
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
    namedWindow("RED FILTER", CV_WINDOW_AUTOSIZE);
    imshow("RED FILTER", filtered);
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
    // Mat image_red;

    cout << "\nConnect a webcam in the USB..." << endl;
    cout << "\nPress ESC to exit()..." << endl;

    while (true) {
      cap->read(image);
      // image_red = ;

      // filtered_yellow = filter_yellow(image);

      // namedWindow("YELLOW_FILTER", CV_WINDOW_AUTOSIZE);
      // imshow("YELLOW_FILTER", image_red);
      // namedWindow("RED_FILTER", CV_WINDOW_AUTOSIZE);
      // imshow("RED_FILTER", filtered_yellow);
      // namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);
      imshow(WINDOW_NAME, detectGREEN(detectYELLOW(detectRED(image))));

      if (waitKey(15) == 27 || waitKey(15) == 43) {
        cout << "\nESC key is pressed by user" << endl;
        break;
      }
    }
  #endif
#endif
}
