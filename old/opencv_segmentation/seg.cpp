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

#define SDRAM_TESTE_BASE 0x30000000
#define SDRAM_TESTE_SPAN 0x4B000

#define HW_REGS_BASE ALT_STM_OFST
#define HW_REGS_SPAN 0x04000000
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

using namespace cv;
using namespace std;


Mat image;
char window_name[] = "CRIOS Teste";

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

unsigned char* getRawImageSDRAM(){
  int fd;
  void *sdram_value;
  if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( (unsigned char*)1 );
  }
  sdram_value = mmap( NULL, SDRAM_TESTE_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_TESTE_BASE );


  // HexDump((unsigned char*)sdram_value, SDRAM_TESTE_SPAN);
  return (unsigned char*)sdram_value;
}

int main(int argc, char** argv) {
  unsigned char *imgIndex;
  int fd;
  void *virtual_base,
       *lw_hps2fpga,
       *lw_fpga2hps;

  if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
   printf( "ERROR: could not open \"/dev/mem\"...\n" );
   return( 1 );
  }

  virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, (off_t)HW_REGS_BASE );

  if ( virtual_base == MAP_FAILED ){
    printf("Error: mmap() failed...\n");
    close(fd);
    return (1);
  }

  lw_hps2fpga = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_OUTPUT_2_FPGA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
  lw_fpga2hps = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_INPUT_2_HPS_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

  while (true){
      *(uint32_t *)lw_hps2fpga = 0x1; // Resquest Image
      bool state = (*(uint32_t *)lw_fpga2hps & 0xffffffff) > 0 ? true : false;
      while (state) printf("Aguardando FPGA...\n");
      // printf("\nVALOR DO LW FPGA->HPS: %x",(lw_fpga2hps));
      imgIndex = getRawImageSDRAM();
      Mat img(480, 640, CV_8UC1, imgIndex);
      namedWindow(window_name, CV_WINDOW_AUTOSIZE );
      imshow(window_name,img);
      if (waitKey(1) == 27) {
        cout << "esc key is pressed by user" << endl;
        break;
      }
  }
}
