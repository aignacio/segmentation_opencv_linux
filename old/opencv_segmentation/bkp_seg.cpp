#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

#define SDRAM_TESTE_BASE 0x30000000
#define SDRAM_TESTE_SPAN 0x4B000

using namespace cv;
using namespace std;

Mat image, imageHSV;
char window_name[] = "CRIOS Teste";

const int w = 500;
int levels = 3;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

void HexDump( unsigned char * buffer, int bytes)
{
  int i;
  for(i=0; i<bytes; i++) {
    if(i%16 == 0)
    printf("\n [%04X] " , i);
    printf("%02X " , buffer[i]);
  }
  printf( "\n");
}

unsigned char* getRawImageSDRAM(){
  int fd;
  void *sdram_value;
  if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( (unsigned char*)1 );
  }
  sdram_value = mmap( NULL, SDRAM_TESTE_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_TESTE_BASE );

  if( sdram_value == MAP_FAILED ) {
    printf( "ERROR: mmap() failed...\n" );
    close( fd );
    return( (unsigned char*)1 );
  }

  // HexDump((unsigned char*)sdram_value, SDRAM_TESTE_SPAN);


  return (unsigned char*)sdram_value;
}

Mat filter_red(Mat image){
  Mat mask_1, mask_2;
  int th_red = 10;
  cvtColor(image, imageHSV, CV_BGR2HSV);
  inRange(imageHSV, Scalar(0, 150, 150), Scalar(th_red, 255, 255), mask_1);
  inRange(imageHSV, Scalar(180-th_red, 150, 150), Scalar(180, 255, 255), mask_2);
  return mask_1|mask_2;
}

int main(int argc, char** argv) {
  // vector<unsigned char> ImVec(imgIndex, imgIndex + SDRAM_TESTE_SPAN);
  // Mat img = imdecode(ImVec, CV_LOAD_IMAGE_GRAYSCALE);
  // printf("Image(0)=%x %x Size = %d\n\n",ImVec[0],ImVec[1],ImVec.size());
  unsigned char *imgIndex = getRawImageSDRAM();
  char *file_input = argv[1];
  VideoCapture *cap = new VideoCapture(0);

  Mat img(480, 640, CV_8UC1, imgIndex);
  Mat image, filtered, final_img;

  // image = imread(file_input, CV_LOAD_IMAGE_COLOR);
  namedWindow("Image from SDRAM", CV_WINDOW_AUTOSIZE );
  imshow("Image from SDRAM",img);

  if (!cap->isOpened()) {
      return -1;
  }

  while (1) {
    bool test = cap->read(image);
    filtered = filter_red(image);

    Rect bounding_rect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours( filtered, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    for( unsigned int i = 0; i < contours.size(); i++ ) {
      double a = contourArea( contours[i],false);
      bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
      if (a > 1){
        rectangle(image, bounding_rect, Scalar(0,255,0),2, 8,0);
        printf("\nAREA=%f",a);
      }
    }
    // imshow(window_name , filtered);
    imshow("CRIOS", image);
    if (waitKey(1) == 27) {
      cout << "esc key is pressed by user" << endl;
      break;
    }

  }

}
