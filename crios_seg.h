#ifndef _CRIOS_SEG_H_
#define _CRIOS_SEG_H_

// cat /proc/iomem output:
// 00000000-2fffffff : System RAM
//   00008000-00961b43 : Kernel code
//   009ea000-00a9048f : Kernel data
// ff201000-ff201007 : /sopc@0/bridge@0xc0000000/sysid@0x100001000
// ff231000-ff23107f : /sopc@0/bridge@0xc0000000/vip@0x100031000
// ff702000-ff703fff : /sopc@0/ethernet@0xff702000
// ff704000-ff704fff : /sopc@0/flash@0xff704000
// ff708000-ff7080ff : /sopc@0/gpio@0xff708000
// ff709000-ff7090ff : /sopc@0/gpio@0xff709000
// ff70a000-ff70a0ff : /sopc@0/gpio@0xff70a000
// ffb40000-ffb7ffff : /sopc@0/usb@0xffb40000
// ffc02000-ffc0201f : serial
// ffc04000-ffc040ff : /sopc@0/i2c@0xffc04000
// ffc05000-ffc050ff : /sopc@0/i2c@0xffc05000
// ffd02000-ffd020ff : /sopc@0/timer@0xffd02000
// ffd05000-ffd050ff : /sopc@0/rstmgr@0xffd05000
// ffe01000-ffe01fff : /sopc@0/dma@0xffe01000
// fff01000-fff010ff : /sopc@0/spi@0xfff01000

#define SDRAM_SEGMENTED_BASE 0x30000000 // This address is mapped out of RAM/Linux Range - 805.306368 MB
#define SDRAM_SEGMENTED_SPAN 0x4B000    // 4B000 - 307200 pixels-bytes

#define HW_REGS_BASE ALT_STM_OFST
#define HW_REGS_SPAN 0x04000000
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define MIN_AREA_DETECT 800
#define WINDOW_NAME "CRIOS Test"

#define SDRAM        1
#define WEBCAM       2
#define IMG_INPUT    3

#define INPUT_SOURCE SDRAM

#ifndef INPUT_SOURCE
  #error "Define a input source MACRO in the crios_seg.h"
#endif

#if INPUT_SOURCE == (IMG_INPUT || WEBCAM)
  #warning "IMAGE - In this case you can use a colored image because the program will filter the RED color to you!"
#endif

#define ENABLE_SEGMENTATION
#define EN_REQ_WRITE    alt_write_word(lw_hps2fpga, 0x01)
#define DIS_REQ_WRITE   alt_write_word(lw_hps2fpga, 0x00)
#define INPUT_FPGA_BIT0 (alt_read_word(lw_fpga2hps)&0x1)
#define WAIT_DONE_WRITE (INPUT_FPGA_BIT0 == 0x0)

#endif
