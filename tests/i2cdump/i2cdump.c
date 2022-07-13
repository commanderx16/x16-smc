#include <stdio.h>
#include <cbm.h>

#define SMC 0x42
#define KEYBUFFER 7

extern int __fastcall__ cx16_k_i2c_readbyte(char offset, char device);
extern char __fastcall__ cx16_k_i2c_writebyte(char value, char offset, char device);


void main() {
  static int k;
  static char pause = 0;
  VERA.irq_enable = 0;
  printf("%c",CH_HOME);
  while (1) {
    k=cx16_k_i2c_readbyte(KEYBUFFER,SMC);
    if (k<0) {
      printf("!");
    }
    else if (k==0) {
      if (!pause) {
        pause=1;
        printf(".");
      }
    }
    else {
      pause=0;
      printf("%02x,",k);
    }
  }
}
